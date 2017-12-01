/**
 * VideoProcessing class is used to do all the vision processing for the mission. This includes
 * circle tracking, rectangle tracking, quadrilateral tracking, as well as many calculations
 * for calculating information such as finding the slope, radius etc. 
 **/

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Threading;
using System.IO;
using System.Reflection;
using System.Drawing.Imaging;
using System.Collections;
using System.Security.Cryptography;

using AForge.Video;
using AForge.Video.DirectShow;
using AForge.Imaging;
using AForge.Imaging.Filters;
using AForge.Controls;
using AForge.Math;
using AForge.Fuzzy;
using AForge.Genetic;
using AForge.MachineLearning;
using AForge.Math.Geometry;
using AForge;

namespace Project_AUV_2015
{
    class VideoProcessing
    {
        // Line needed to check for shapes
        public SimpleShapeChecker shapeChecker = new SimpleShapeChecker();

        // Quadrilateral/Slope Tracking variables
        public AForge.Point one;
        public AForge.Point two;
        public AForge.Point three;
        public AForge.Point four;
        int mostWidth = 0;
        int arrayNum = 0;
        int leastX = 1000;
        int mostX = 0;

        // Circle Tracking Variables
        //public DoublePoint center;
        //public double radius;

        // Rectangle Tracking Variables
        public Rectangle rect;
        public Blob[] rectBlob;

        // Slope Tracking Variables
        double[] slopeArray = new double[10];
        public double slope;
        public double slopeCenterX;
        public double slopeCenterY;

        public bool rectInView = false;
        public bool slopeInView = false;
        public Blob[] blobs;

        /**
         * Slope Tracking which gets a video image and finds the slope of any line it sees
         * on the screen. It does this by first tracking a quadrilateral and then finding
         * the slope of the longest side. 
         **/ 
        public void SlopeTracking(Bitmap image, Bitmap image2)
        {
            BlobCounter blobCounter = new BlobCounter();
            blobCounter.FilterBlobs = true;
            blobCounter.MinHeight = 75;
            blobCounter.MinWidth = 25;
            //blobCounter.MaxHeight = 240;
            //blobCounter.MaxWidth = 320;
            blobCounter.ProcessImage(image);
            blobs = blobCounter.GetObjectsInformation();

            Graphics g = Graphics.FromImage(image2);
            Pen bluePen = new Pen(Color.FloralWhite, 5);
            if (blobs.Length == 0) slopeInView = false;
            if (blobs.Length == 1)
            {
                List<IntPoint> edgePoints = blobCounter.GetBlobsEdgePoints(blobs[0]);
                List<IntPoint> corners = PointsCloud.FindQuadrilateralCorners(edgePoints);

                if (shapeChecker.IsQuadrilateral(edgePoints, out corners))
                {
                    // Draw Quadrilateral
                    slopeInView = true;
                    g.DrawPolygon(bluePen, ToPointsArray(corners));

                    one = new AForge.Point(ToPointsArray(corners)[0].X, ToPointsArray(corners)[0].Y);
                    two = new AForge.Point(ToPointsArray(corners)[1].X, ToPointsArray(corners)[1].Y);
                    three = new AForge.Point(ToPointsArray(corners)[2].X, ToPointsArray(corners)[2].Y);
                    four = new AForge.Point(ToPointsArray(corners)[3].X, ToPointsArray(corners)[3].Y);

                    findSlope();

                    slope1 = slope;
                    slopeCenterX1 = slopeCenterX;
                    slopeCenterY1 = slopeCenterY;
                }
                else
                {
                    slopeInView = false;
                }
            }
            else if (blobs.Length == 2)
            {
                List<IntPoint> edgePoints = blobCounter.GetBlobsEdgePoints(blobs[0]);
                List<IntPoint> corners = PointsCloud.FindQuadrilateralCorners(edgePoints);

                if (shapeChecker.IsQuadrilateral(edgePoints, out corners))
                {
                    // Draw Quadrilateral
                    slopeInView = true;
                    g.DrawPolygon(bluePen, ToPointsArray(corners));

                    one = new AForge.Point(ToPointsArray(corners)[0].X, ToPointsArray(corners)[0].Y);
                    two = new AForge.Point(ToPointsArray(corners)[1].X, ToPointsArray(corners)[1].Y);
                    three = new AForge.Point(ToPointsArray(corners)[2].X, ToPointsArray(corners)[2].Y);
                    four = new AForge.Point(ToPointsArray(corners)[3].X, ToPointsArray(corners)[3].Y);

                    findSlope();
                    slope1 = slope;
                    slopeCenterX1 = slopeCenterX;
                    slopeCenterY1 = slopeCenterY;
                }
                else
                {
                    slopeInView = false;
                }

                edgePoints = blobCounter.GetBlobsEdgePoints(blobs[1]);
                corners = PointsCloud.FindQuadrilateralCorners(edgePoints);

                if (shapeChecker.IsQuadrilateral(edgePoints, out corners))
                {
                    // Draw Quadrilateral
                    slopeInView = true;
                    g.DrawPolygon(bluePen, ToPointsArray(corners));

                    one = new AForge.Point(ToPointsArray(corners)[0].X, ToPointsArray(corners)[0].Y);
                    two = new AForge.Point(ToPointsArray(corners)[1].X, ToPointsArray(corners)[1].Y);
                    three = new AForge.Point(ToPointsArray(corners)[2].X, ToPointsArray(corners)[2].Y);
                    four = new AForge.Point(ToPointsArray(corners)[3].X, ToPointsArray(corners)[3].Y);

                    findSlope();
                    slope2 = slope;
                    slopeCenterX2 = slopeCenterX;
                    slopeCenterY2 = slopeCenterY;
                }
                else
                {
                    slopeInView = false;
                }
            }

            bluePen.Dispose();
            g.Dispose();
        }

        public double slope1 = 0;
        public double slope2 = 0;
        public double slopeCenterX1 = 0;
        public double slopeCenterY1 = 0;
        public double slopeCenterX2 = 0;
        public double slopeCenterY2 = 0;


        public void findSlope()
        {
            double line1;
            double line2;
            double line1Point1X;
            double line1Point1Y;
            double line1Point2X;
            double line1Point2Y;
            double line2Point1X;
            double line2Point1Y;
            double line2Point2X;
            double line2Point2Y;
            double slopeLine;
            double slopePoint1X;
            double slopePoint1Y;
            double slopePoint2X;
            double slopePoint2Y;

            double distance12 = Math.Sqrt(Math.Pow(two.Y - one.Y, 2) + Math.Pow(two.X - one.X, 2));
            double distance23 = Math.Sqrt(Math.Pow(three.Y - two.Y, 2) + Math.Pow(three.X - two.X, 2));
            double distance34 = Math.Sqrt(Math.Pow(four.Y - three.Y, 2) + Math.Pow(four.X - three.X, 2));
            double distance41 = Math.Sqrt(Math.Pow(one.Y - four.Y, 2) + Math.Pow(one.X - four.X, 2));

            // find center
            slopeCenterX = (one.X + two.X + three.X + four.X) / 4;
            slopeCenterY = (one.Y + two.Y + three.Y + four.Y) / 4;

            // find slope
            if (distance12 - distance23 < 25)
            {
                line1 = distance23;
                line1Point1X = two.X;
                line1Point1Y = two.Y;
                line1Point2X = three.X;
                line1Point2Y = three.Y;
            }
            else
            {
                line1 = distance12;
                line1Point1X = one.X;
                line1Point1Y = one.Y;
                line1Point2X = two.X;
                line1Point2Y = two.Y;
            }

            if (line1 == distance12)
            {
                line2 = distance34;
                line2Point1X = one.X;
                line2Point1Y = one.Y;
                line2Point2X = two.X;
                line2Point2Y = two.Y;
            }
            else
            {
                line2 = distance41;
                line2Point1X = four.X;
                line2Point1Y = four.Y;
                line2Point2X = one.X;
                line2Point2Y = one.Y;
            }

            if (line1Point1X < line2Point1X)
            {
                slopeLine = line1;
                slopePoint1X = line1Point1X;
                slopePoint1Y = line1Point1Y;
                slopePoint2X = line1Point2X;
                slopePoint2Y = line1Point2Y;
            }
            else
            {
                slopeLine = line2;
                slopePoint1X = line2Point1X;
                slopePoint1Y = line2Point1Y;
                slopePoint2X = line2Point2X;
                slopePoint2Y = line2Point2Y;
            }
            slope = (slopePoint2Y - slopePoint1Y) / (slopePoint2X - slopePoint1X);
            slope = Math.Round(slope, 2);
            slope = Math.Atan(1 / slope) * 180 / Math.PI;
        }

        /**
         * Turn corners of quadrilateral from slope tracking method into points.
         **/
        private System.Drawing.Point[] ToPointsArray(List<IntPoint> points)
        {
            System.Drawing.Point[] array = new System.Drawing.Point[points.Count];

            for (int i = 0, n = points.Count; i < n; i++)
            {
                array[i] = new System.Drawing.Point(points[i].X, points[i].Y);
            }

            return array;
        }

        /**
         * Circle tracking method for tracking circles.
         **/
        public void CircleTracking(Bitmap image, Bitmap image2)
        {
            // lock image
            BitmapData bitmapData = image.LockBits(new Rectangle(0, 0, image.Width, image.Height),ImageLockMode.ReadWrite, image.PixelFormat);

            BlobCounter blobCounter = new BlobCounter();
            blobCounter.FilterBlobs = true;
            blobCounter.MinHeight = 100;
            blobCounter.MinWidth = 100;

            blobCounter.ProcessImage(bitmapData);
            Blob[] blobs = blobCounter.GetObjectsInformation();
            image.UnlockBits(bitmapData);

            // Set the color of circle to be drawn
            Graphics g = Graphics.FromImage(image2);
            Pen yellowPen = new Pen(Color.Yellow, 4);

            // Check to see if object is a circle
            for (int i = 0, n = blobs.Length; i < n; i++)
            {
                List<IntPoint> edgePoints = blobCounter.GetBlobsEdgePoints(blobs[i]);

                // Draw a yellow circle around all circles in the image. Gives the value of center
                // and radius in if statement
                //if (shapeChecker.IsCircle(edgePoints, out center, out radius))
                //{
                //    g.DrawEllipse(yellowPen, (float)(center.X - radius), (float)(center.Y - radius), (float)(radius * 2), (float)(radius * 2));
                //}
            }
            // Erase circle
            yellowPen.Dispose();
        }
        public Rectangle[] rects;
        /**
         * Rectangle tracking method for tracking rectangles.
         **/
        public void RectangleTracking(Bitmap image, Bitmap image2, int minWidth)
        {
            // lock image
            BitmapData bitmapData = image.LockBits(new Rectangle(0, 0, image.Width, image.Height), ImageLockMode.ReadWrite, image.PixelFormat);

            BlobCounter blobCounter1 = new BlobCounter();
            blobCounter1.FilterBlobs = true;
            blobCounter1.MinHeight = 30;
            blobCounter1.MinWidth = 30;
            blobCounter1.MaxHeight = 400;
            blobCounter1.MaxWidth = 600;

            blobCounter1.ProcessImage(bitmapData);
            rectBlob = blobCounter1.GetObjectsInformation();
            image.UnlockBits(bitmapData);

            rects = blobCounter1.GetObjectsRectangles();

            if (rects.Length > 0)
            {

                //For Right
                //for (int i = 0; i < rects.Length; i++)
                //{
                //    rect = rects[i];
                //    if (rect.X > mostX)
                //    {
                //        mostX = rect.X;
                //        arrayNum = i;
                //    }
                //}
                //rect = rects[arrayNum];

                //For Width
                //for (int i = 0; i < rects.Length; i++)
                //{
                //    rect = rects[i];
                //    if (rect.Width > mostWidth)
                //    {
                //        mostWidth = rect.Width;
                //        arrayNum = i;
                //    }
                //}
                //rect = rects[arrayNum];

                //For Left
                for (int i = 0; i < rects.Length; i++)
                {
                    rect = rects[i];
                    if (rect.X < leastX)
                    {
                        leastX = rect.X;
                        arrayNum = i;
                    }
                }
                rect = rects[arrayNum];
                

                rectInView = true;

                Graphics g1 = Graphics.FromImage(image2);

                using (Pen pen = new Pen(Color.FromArgb(160, 255, 160), 4))
                {
                    if ((rect.Size.Height > 5) && (rect.Size.Width > 5))
                    {
                        g1.DrawRectangle(pen, rect);
                    }
                }
            }
            else
            {
                rectInView = false;
            }
            leastX = 1000;
            mostWidth = 0;
            arrayNum = 0;
            mostX = 0;

        }
    }
}
