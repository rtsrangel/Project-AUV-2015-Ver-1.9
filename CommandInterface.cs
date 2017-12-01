/**
 * Project AUV 2012
 * By John Rangel, Sergio Corral, John Harris, Borna Emami, and Carlos Ruvio
 * 
 * Description:
 * This code is to be used on the Carl Hayden High School Falcon Robotics
 * team for their autonomous underwater vehicle. This code includes
 * image processing, motor control, as well as an interface to communicate
 * with the robot. For our project we use sample code from many different sources 
 * specifically the Phidget libraries and the AForge.NET Framework. 
 **/
// Call all default variables
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Threading;
using System.IO;
using System.IO.Ports;
using System.Reflection;
using System.Drawing.Imaging;

// Call all AForge variables
using AForge.Imaging.Filters;
using AForge.Video.VFW;
using AForge.Video;
using AForge.Video.DirectShow;
using AForge.Imaging;
using AForge.Controls;
using AForge.Math;
using AForge.Fuzzy;
using AForge.Genetic;
using AForge.MachineLearning;
using AForge.Math.Geometry;
using AForge;

// Call EmguCV variables
using Emgu.CV;
using Emgu.Util;
using Emgu.CV.Structure;

// Reference to phidget libraries
using Phidgets;        
using Phidgets.Events;

// Reference to joystick libraries
using SlimDX;
using SlimDX.DirectInput;
using System.Globalization;

namespace Project_AUV_2015
{
    public partial class CommandInterface : Form
    {
        /**
         * Area where all variables are defined. Every variable is made public to allow easy access
         * from other classes in the code.
         **/
        // Video Stream Variables
        public FilterInfoCollection VideoCaptureDevices;
        public VideoCaptureDevice FrontVideoSource;
        public VideoCaptureDevice BottomVideoSource;
         
        // PID Variables
        PID pidHeading = new PID();
        PID pidDepth = new PID();
        PID pidX = new PID();
        PID pidY = new PID();
        PID pidSlope = new PID();
        PID pidFCam = new PID();

        // Video Displaying Variables
        public Bitmap FrontVideoImage;
        public Bitmap BottomVideoImage;
        public int FVideoCenterX;
        public int BVideoCenterX;

        // Color Filtering Variables
        private int lowHue = 0;
        private int highHue = 400;
        public float lowSat = 0;
        public float highSat = 1;
        public float lowLum = 0;
        public float highLum = 1;
        public Boolean hslOn = false;

        // Circle Tracking Variables
        public DoublePoint FCircleCenter;
        public double FCircleRadius;
        public DoublePoint BCircleCenter;
        public double BCircleRadius;
        public Boolean ramReady;
        public Boolean ramDone;
        public Boolean locked;
        public Boolean gyroReady;

        // Rectangle Tracking Variables
        public Rectangle FRectangle;
        public Rectangle BRectangle;
        public double rectCenterX;
        public double rectCenterY;
        public int minWidth = 70;
        int rectCount;
        double rectTestX;
        double rectTestY;
        DateTime timeL;

        // Slope Tracking Variables
        public AForge.Point[] rectPoints = new AForge.Point[4];
        public AForge.Point pFOne;
        public AForge.Point pFTwo;
        public AForge.Point pFThree;
        public AForge.Point pFFour;
        public AForge.Point pBOne;
        public AForge.Point pBTwo;
        public AForge.Point pBThree;
        public AForge.Point pBFour;
        int slopeCount = 0;

        // Data logger variables
        public int dataRefreshCount = 0;
        public int dataRefreshSecond = 0;
        public int dataRefreshMinute = 0;
        public String time = "00:00\t";

        // Create a video processing object to use vision processing methods
        VideoProcessing videoProcessing = new VideoProcessing();

        // Interface Kit Variables
        private InterfaceKit ifKit;
        private CheckBox[] digiInArray = new CheckBox[16];
        private CheckBox[] digiOutArray = new CheckBox[16];
        private CheckBox[] digiOutDispArray = new CheckBox[16];
        private TextBox[] sensorInArray = new TextBox[8];
        InterfaceKit_full.AdvancedSensorForm advSensorForm;
        Boolean sensors = false;
        
        // Motor Controller Variables
        private MotorControl fRMotorControl;
        private MotorControl fLMotorControl;
        private MotorControl bRMotorControl;
        private MotorControl bLMotorControl;
        private MotorControl rVMotorControl;
        private MotorControl lVMotorControl;
        int secondCount;
        int seconds;

        int flood1Num1 = 75;
        int flood1Num2 = 92;
        int flood1Num3 = 92;
        int flood2Num1 = 250;
        int flood2Num2 = 125;
        int flood2Num3 = 125;
        int flood3Num1 = 75;
        int flood3Num2 = 92;
        int flood3Num3 = 92;
        int flood4Num1 = 75;
        int flood4Num2 = 92;
        int flood4Num3 = 92;

        double leftTime = 13000;
        double direction = -50;

        public CommandInterface()
        {
            InitializeComponent();
            pidHeading.PIDinit(1,0,0, -100, 100, 1, 1);
            pidDepth.PIDinit(1,0,0, -100, 100, 15, 1);
            pidX.PIDinit(1,0,0, -100, 100, 1, 1);
            pidY.PIDinit(1,0,0, -100, 100, 1, 1);
            pidSlope.PIDinit(1, 0, 0, -100, 100, 1, 1);
            pidFCam.PIDinit(1, 0, 0, -100, 100, 1, 1);
            for(int i = 0; i <11 ; i++)
                dataGridView1.Rows.Add();
            string[] names = {"kpHeading","kdHeading","kpDepth","kdDepth","kiDepth","kpPos","kdPos","kpSlope","kdSlope","kpFcam","kdFcam"};
            //string[] name = new string[];
            string[] values = Project_AUV_2015.Properties.Settings.Default.PIDvals.Split(',');
            for (int i = 0; i < 11; i++)
            {
                dataGridView1.Rows[i].Cells[0].Value = names[i];
                
            }
            for (int i = 0; i < values.Length; i++)
            {
                dataGridView1.Rows[i].Cells[1].Value = values[i];
            }
        }

        /**
         * Loading of the command interface. When it loads it also sets any nessasary
         * paramaters as well as starts any timers or methods needed to run code. 
         **/
        private void CommandInterface_Load(object sender, EventArgs e)
        {
            //chooseBuoy.SelectedIndex = 0;
            //panel1.BackColor = Color.FromArgb(100, Color.SteelBlue);
            //panel2.BackColor = Color.FromArgb(100, Color.SteelBlue);
            //panel3.BackColor = Color.FromArgb(100, Color.SteelBlue);
            // Get information on all video devices connected to the computer

            VideoCaptureDevices = new FilterInfoCollection(FilterCategory.VideoInputDevice);

            // Add all video sources to the combo boxes
            foreach (FilterInfo VideoCaptureDevice in VideoCaptureDevices)
            {
                FrontVideoComboBox.Items.Add(VideoCaptureDevice.Name);
                BottomVideoComboBox.Items.Add(VideoCaptureDevice.Name);
            }
            
            // Automatically select cameras for displaying video based on order as they appear
            FrontVideoComboBox.SelectedIndex = 0;
            //BottomVideoComboBox.SelectedIndex = 1;
            
            // Set delay before starting to capture video
            System.Threading.Thread.Sleep(1000);

            // Start capturing video from both cameras
            FrontVideoSource = new VideoCaptureDevice(VideoCaptureDevices[FrontVideoComboBox.SelectedIndex].MonikerString);
            FrontVideoSource.NewFrame += new NewFrameEventHandler(FrontVideoRefresh);
            FrontVideoSource.DesiredFrameSize = new Size(320,240);
            FrontVideoSource.DesiredFrameRate = 15;
            FrontVideoSource.Start();
            System.Threading.Thread.Sleep(1000);
            //BottomVideoSource = new VideoCaptureDevice(VideoCaptureDevices[BottomVideoComboBox.SelectedIndex].MonikerString);
            //BottomVideoSource.NewFrame += new NewFrameEventHandler(BottomVideoRefresh);
            //BottomVideoSource.DesiredFrameSize = new Size(320, 240);
            //BottomVideoSource.DesiredFrameRate = 15;
            //BottomVideoSource.Start();

            /*
            // Turn on the refresh data timer which constantly refreshes the command interface with new data from vision, phidgets, and other parts of the code
            RefreshData.Enabled = true;

            // Phidget opening motor code
            fRMotorControl = new MotorControl();
            fLMotorControl = new MotorControl();
            bRMotorControl = new MotorControl();
            bLMotorControl = new MotorControl();
            rVMotorControl = new MotorControl();
            lVMotorControl = new MotorControl();
            fRMotorControl.open(142947);
            fLMotorControl.open(146704);
            bRMotorControl.open(146765);
            bLMotorControl.open(146819);
            rVMotorControl.open(146576);
            lVMotorControl.open(142964);

            // Enable Joystick Control
            BootGamePort();
            */
            // Phidget Compass Opening Code
            Phidgets.openCompass();
            

            // Phidget Interface Kit opening code
            makeDigiInArray();
            makeDigiOutArray();
            makeSensorInArray();

            label8.Visible = false;
            inputTrk.Value = 0;
            inputTrk.Enabled = false;
            inputTrk.Visible = false;
            sensitivityTxt.Text = "";
            sensitivityTxt.Visible = false;

            ratioChk.Enabled = false;
            ratioChk.Checked = false;
            ratioChk.Visible = false;
            sensorsButton.Visible = false;

            try
            {
                ifKit = new InterfaceKit();

                ifKit.Attach += new AttachEventHandler(ifKit_Attach);
                ifKit.Detach += new DetachEventHandler(ifKit_Detach);
                ifKit.InputChange += new InputChangeEventHandler(ifKit_InputChange);
                ifKit.SensorChange += new SensorChangeEventHandler(ifKit_SensorChange);
                ifKit.open(343667);
            }
            catch (PhidgetException ex)
            {
                MessageBox.Show(ex.ToString());
            }
        }
        
        /**
         * The closing of the command interface closes all video feeds, motors,
         * and sensors. 
         **/
        private void CommandInterface_FormClosing(object sender, FormClosingEventArgs e)
        {
            // Stop all video sources from running and close them
            FrontVideoSource.Stop();
            //BottomVideoSource.Stop();
            
            // Phidget Closing Code
            ifKit.Attach -= new AttachEventHandler(ifKit_Attach);
            ifKit.Detach -= new DetachEventHandler(ifKit_Detach);
            ifKit.InputChange -= new InputChangeEventHandler(ifKit_InputChange);
            ifKit.SensorChange -= new SensorChangeEventHandler(ifKit_SensorChange);

            // Turn off Mission Planner
            MissionPlanner.Enabled = false;
            /*
            if (fRMotorControl.Attached)
            {
                fRMotorControl.motors[0].Velocity = 0;
            }
            if (fLMotorControl.Attached)
            {
                fLMotorControl.motors[0].Velocity = 0;
            }
            if (bRMotorControl.Attached)
            {
                bRMotorControl.motors[0].Velocity = 0;
            }
            if (bLMotorControl.Attached)
            {
                bLMotorControl.motors[0].Velocity = 0;
            }
            if (rVMotorControl.Attached)
            {
                rVMotorControl.motors[0].Velocity = 0;
            }
            if (lVMotorControl.Attached)
            {
                lVMotorControl.motors[0].Velocity = 0;
            }
            System.Threading.Thread.Sleep(1000);
            */
            Phidgets.closeCompass();

            ////run any events in the message queue - otherwise close will hang if there are any outstanding events
            Application.DoEvents();
            
            ifKit.close();
            /*
            fRMotorControl.close();
            fLMotorControl.close();
            bRMotorControl.close();
            bLMotorControl.close();
            rVMotorControl.close();
            lVMotorControl.close();
            fRMotorControl = null;
            fLMotorControl = null;
            bRMotorControl = null;
            bLMotorControl = null;
            rVMotorControl = null;
            lVMotorControl = null;
             */
        }

        int pictureNumber = 0;
        Bitmap video1;
        Bitmap video2;
        Bitmap video3;

        /*******************************************************************************************************/
        void trackQuadrilaterals(Bitmap frame, Bitmap hslImage)
	    {
            Image<Bgr, Byte> normalImage = new Image<Bgr, Byte>(frame);
            Image<Gray, Byte> grayImage = new Image<Gray, Byte>(hslImage);
            Image<Bgr, Byte> trackingImage = new Image<Bgr, Byte>(frame); //where bmp is a Bitmap

            //grayImage = grayImage.ThresholdBinary(new Gray(125), new Gray(255));

            //if (invert)
            //{
            //    grayImage._Not();
            //}  

            using (MemStorage storage = new MemStorage())
            {
                for (Contour<System.Drawing.Point> contours = grayImage.FindContours(Emgu.CV.CvEnum.CHAIN_APPROX_METHOD.CV_CHAIN_APPROX_SIMPLE, Emgu.CV.CvEnum.RETR_TYPE.CV_RETR_TREE, storage); contours != null; contours = contours.HNext)
                {
                    Contour<System.Drawing.Point> currentContour = contours.ApproxPoly(contours.Perimeter * 0.015, storage);
                    if (currentContour.BoundingRectangle.Width > 20)
                    {
                        CvInvoke.cvDrawContours(trackingImage, contours, new MCvScalar(255), new MCvScalar(255), -1, 2, Emgu.CV.CvEnum.LINE_TYPE.EIGHT_CONNECTED, new System.Drawing.Point(0, 0));
                        //trackingImage.Draw(currentContour.BoundingRectangle, new Bgr(0, 255, 0), 1);
                    }
                }

            } 

		    //vector<Vec4i> hierarchy;
		    //vector<vector<Point> > contours;
            ////CvInvoke.cvCanny(src_gray, src_gray, 50, 200, 3);

            ////int[,] tempStructure = { { 1, 1, 1 } };
            ////StructuringElementEx element = new StructuringElementEx(tempStructure, 1, 0);
            ////CvInvoke.cvDilate(src_gray, src_gray, element, 2);
            //CvInvoke.cvFindContours();
            /*
            //finding all contours in the image
            findContours(src_gray, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

            // Approximate contours to polygons + get bounding rects and circles
            vector<vector<Point> > contours_poly(contours.size());
            vector<Rect> boundRect(contours.size());
            int col;
            /// Draw contours
            Mat drawing = Mat::zeros(src_gray.size(), CV_8UC3);
            for (int i = 0; i < contours.size(); i++)
            {
                approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
                drawContours(drawing, contours_poly, i, Scalar(0, 0, 255), 1, 8, vector<Vec4i>(), 0, Point() );
                boundRect[i] = boundingRect( Mat(contours_poly[i]));

                rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), Scalar(255, 0, 255), 2, 8, 0 );
            }
            */
            video1 = normalImage.ToBitmap();
            video2 = grayImage.ToBitmap();
            video3 = trackingImage.ToBitmap();
	    }

        /*************************************************************************************************/
        //C:\Users\krazy_000\Pictures
        static String binSource = "C:/Users/krazy_000/Pictures/bins.png";
        //Uncomment for pictures
        Bitmap bin1 = (Bitmap)System.Drawing.Image.FromFile(binSource);
        Bitmap bin2 = (Bitmap)System.Drawing.Image.FromFile(binSource);
        Bitmap bin3 = (Bitmap)System.Drawing.Image.FromFile(binSource);

        /**
         * Displays video on the command interface and uses data from the VideoProcessing class
         **/
        private void FrontVideoRefresh(object sender, NewFrameEventArgs eventArgs)
        {
            //Uncomment for pictures
            //String source = "C:/Users/krazy_000/Pictures/bins.png";
            //Bitmap image1 = (Bitmap)System.Drawing.Image.FromFile(source);
            //Bitmap image2 = (Bitmap)System.Drawing.Image.FromFile(source);
            //Bitmap image3 = (Bitmap)System.Drawing.Image.FromFile(source);
            //FrontVideoImage = (Bitmap)bin1.Clone();
            //Bitmap hslFrontImage = (Bitmap)bin2.Clone();
            //Bitmap FTrackingImage = (Bitmap)bin3.Clone();
            
            //Uncomment for Video
            FrontVideoImage = (Bitmap)eventArgs.Frame.Clone();
            Bitmap hslFrontImage = (Bitmap)eventArgs.Frame.Clone();
            Bitmap FTrackingImage = (Bitmap)eventArgs.Frame.Clone();

            // Create filter for resizing video images
            ResizeNearestNeighbor resizeFilter = new ResizeNearestNeighbor(320, 240);
            ResizeNearestNeighbor lowerResolution = new ResizeNearestNeighbor(160, 120);

            // Apply the resize filter
            Bitmap resizedFVideoImage = resizeFilter.Apply(FrontVideoImage);
            Bitmap resizedhslImage = resizeFilter.Apply(hslFrontImage);
            Bitmap resizedFTrackingImage = resizeFilter.Apply(FTrackingImage);
            
            // Turn hsl filter on if the hsl checkbox is checked
            if (hslOn)
            {
                // Make the hsl filter and set the values
                HSLFiltering HSLfilter = new HSLFiltering();

                HSLfilter.Hue = new AForge.IntRange(lowHue, highHue);
                HSLfilter.Saturation = new Range(lowSat, highSat);
                HSLfilter.Luminance = new Range(lowLum, highLum);

                // Apply the hsl filter onto the image
                HSLfilter.ApplyInPlace(hslFrontImage);
            }

            if (floodFillBox.Checked)
            {
                Bitmap floodFill1 = (Bitmap)hslFrontImage.Clone();
                Bitmap floodFill2 = (Bitmap)hslFrontImage.Clone();
                Bitmap floodFill3 = (Bitmap)hslFrontImage.Clone();
                Bitmap floodFill4 = (Bitmap)hslFrontImage.Clone();

                // create filter
                PointedColorFloodFill filter1 = new PointedColorFloodFill();
                // configure the filter
                filter1.Tolerance = Color.FromArgb(flood1Num1, flood1Num2, flood1Num3);
                filter1.FillColor = Color.FromArgb(0, 0, 0);
                filter1.StartingPoint = new IntPoint(10, 10);
                filter1.ApplyInPlace(floodFill1);

                // create filter
                PointedColorFloodFill filter2 = new PointedColorFloodFill();
                // configure the filter
                filter2.Tolerance = Color.FromArgb(flood1Num1, flood1Num2, flood1Num3);
                filter2.FillColor = Color.FromArgb(0, 0, 0);
                filter2.StartingPoint = new IntPoint(310, 10);
                filter2.ApplyInPlace(floodFill2);

                // create filter
                PointedColorFloodFill filter3 = new PointedColorFloodFill();
                // configure the filter
                filter3.Tolerance = Color.FromArgb(50, flood1Num2, flood1Num3);
                filter3.FillColor = Color.FromArgb(0, 0, 0);
                filter3.StartingPoint = new IntPoint(10, 230);
                filter3.ApplyInPlace(floodFill3);

                // create filter
                PointedColorFloodFill filter4 = new PointedColorFloodFill();
                // configure the filter
                filter4.Tolerance = Color.FromArgb(flood1Num1, flood1Num2, flood1Num3);
                filter4.FillColor = Color.FromArgb(0, 0, 0);
                filter4.StartingPoint = new IntPoint(310, 230);
                filter4.ApplyInPlace(floodFill4);

                //hslFrontImage = floodFill1;
                Merge merge1 = new Merge(floodFill1);
                Bitmap mergeOutput1 = merge1.Apply(floodFill2);
                Merge merge2 = new Merge(floodFill3);
                Bitmap mergeOutput2 = merge2.Apply(floodFill4);

                Merge mergeFinal = new Merge(mergeOutput1);
                hslFrontImage = mergeFinal.Apply(mergeOutput2);
                
                //FTrackingImage = hslFrontImage;
               // dilationFilter.Apply(FTrackingImage);
               // erosianFilter.Apply(FTrackingImage);
                
            }

            // Start tracking buoys
            if (FBuoyButton.Checked)
            {
                //videoProcessing.CircleTracking(hslFrontImage, FTrackingImage);
                //FCircleCenter = videoProcessing.center;
                //FCircleRadius = videoProcessing.radius;
            }

            // Start tracking rectangles
            if (FRectangleButton.Checked)
            {
                //floodFill(hslFrontImage);
                
                videoProcessing.RectangleTracking(hslFrontImage, FTrackingImage, minWidth);
                FRectangle = videoProcessing.rect;
                rectCenterX = (FRectangle.Width / 2) + FRectangle.X;
                rectCenterY = (FRectangle.Height / 2) + FRectangle.Y;
            }

            // Start tracking quadrilaterals
            if (FSlopeButton.Checked)
            {
                videoProcessing.SlopeTracking(hslFrontImage, FTrackingImage);
                rectCenterX = videoProcessing.slopeCenterX-320;
                rectCenterY = videoProcessing.slopeCenterY-240;
                pFOne = videoProcessing.one;
                pFTwo = videoProcessing.two;
                pFThree = videoProcessing.three;
                pFFour = videoProcessing.four;
            }

            trackQuadrilaterals(FrontVideoImage, hslFrontImage);
            FrontVideoImage = video1;
            hslFrontImage = video2;
            FTrackingImage = video3;

            // Apply the resize filter
            resizedFVideoImage = resizeFilter.Apply(FrontVideoImage);
            //Bitmap resizedFVideoImageCopy = resizeFilter.Apply(FrontVideoImage);
            resizedhslImage = resizeFilter.Apply(hslFrontImage);
            resizedFTrackingImage = resizeFilter.Apply(FTrackingImage);

            // Display front camera
            try
            {
                FrontCameraNormal.Image = resizedFVideoImage;
                FrontCameraColorFilter.Image = resizedhslImage;
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.ToString());
            }
            

            FrontCameraObjectTracking.Image = resizedFTrackingImage;
            FVideoCenterX = FrontCameraNormal.Size.Width / 4;
            FVideoCenterX = FrontCameraNormal.Size.Height / 4;

            FrontVideoImage = null;
            hslFrontImage = null;
            FTrackingImage = null;
            // Clear all null variable from memmory
            GC.Collect();
        }

        /**
         * Displays video on the command interface and uses data from the VideoProcessing class
         **/
        private void BottomVideoRefresh(object sender, NewFrameEventArgs eventArgs)
        {
            // Create nessasary clone for displaying video
            BottomVideoImage = (Bitmap)eventArgs.Frame.Clone();
            Bitmap hslBottomImage = (Bitmap)eventArgs.Frame.Clone();
            Bitmap BTrackingImage = (Bitmap)eventArgs.Frame.Clone();

            // Turn hsl filter on if the hsl checkbox is checked
            if (hslOn)
            {
                // Make the hsl filter and set values
                HSLFiltering HSLfilter = new HSLFiltering();

                HSLfilter.Hue = new AForge.IntRange(lowHue, highHue);
                HSLfilter.Saturation = new AForge.Range(lowSat, highSat);
                HSLfilter.Luminance = new AForge.Range(lowLum, highLum);

                // Apply the hsl filter onto the image
                HSLfilter.ApplyInPlace(hslBottomImage);
            }

            // Start tracking circles
            if (BBuoyButton.Checked)
            {
                //videoProcessing.CircleTracking(hslBottomImage, BTrackingImage);
                //BCircleCenter = videoProcessing.center;
                //BCircleRadius = videoProcessing.radius;
            }

            // Start tracking rectangles
            if (BRectangleButton.Checked)
            {
                videoProcessing.RectangleTracking(hslBottomImage, BTrackingImage, minWidth);
                BRectangle = videoProcessing.rect;
                rectCenterX = (BRectangle.Width / 2) + BRectangle.X;
                rectCenterY = (BRectangle.Height / 2) + BRectangle.Y;
            }
            
            // Start tracking quadrilaterals
            if (BSlopeButton.Checked)
            {
                videoProcessing.SlopeTracking(hslBottomImage, BTrackingImage);
                pBOne = videoProcessing.one;
                pBTwo = videoProcessing.two;
                pBThree = videoProcessing.three;
                pBFour = videoProcessing.four;
                rectCenterX = videoProcessing.slopeCenterX - 320;
                rectCenterY = videoProcessing.slopeCenterY - 240;
            }

            // Create filter for resizing video images
            ResizeNearestNeighbor resizeFilter = new ResizeNearestNeighbor(320, 240);
            // Apply the resize filter
            Bitmap resizedBVideoImage = resizeFilter.Apply(BottomVideoImage);
            Bitmap resizedBVideoImageCopy = resizeFilter.Apply(BottomVideoImage);
            Bitmap resizedhslImage = resizeFilter.Apply(hslBottomImage);
            Bitmap resizedBTrackingImage = resizeFilter.Apply(BTrackingImage);

            // Display bottom camera
            BottomCameraNormal.Image = resizedBVideoImage;
            BottomCameraColorFilter.Image = resizedhslImage;
            BottomCameraObjectTracking.Image = resizedBTrackingImage;
            BVideoCenterX = BottomCameraNormal.Size.Width / 2;
            BVideoCenterX = BottomCameraNormal.Size.Height / 2;
        }

        public void floodFill(Bitmap displayImage)
        {
            // create filter
            PointedColorFloodFill filter1 = new PointedColorFloodFill();
            // configure the filter
            filter1.Tolerance = Color.FromArgb(150, 92, 92);
            filter1.FillColor = Color.FromArgb(0, 0, 0);
            filter1.StartingPoint = new IntPoint(10, 10);
            filter1.ApplyInPlace(displayImage);
            // create filter
            PointedColorFloodFill filter2 = new PointedColorFloodFill();
            // configure the filter
            filter2.Tolerance = Color.FromArgb(150, 92, 92);
            filter2.FillColor = Color.FromArgb(0, 0, 0);
            filter2.StartingPoint = new IntPoint(270, 10);
            //filter2.ApplyInPlace(displayImage);
        }

        /**
         * Trackbars are used for changing the hue, saturation, and luminescence.
         **/
        private void LowHueTrackbar_Scroll(object sender, EventArgs e)
        {
            HueLabel.Text = "Hue: " + LowHueTrackbar.Value.ToString() + "," + HighHueTrackbar.Value.ToString();
            lowHue = LowHueTrackbar.Value;
        }
        private void HighHueTrackbar_Scroll(object sender, EventArgs e)
        {
            HueLabel.Text = "Hue: " + LowHueTrackbar.Value.ToString() + "," + HighHueTrackbar.Value.ToString();
            highHue = HighHueTrackbar.Value;
        }
        private void LowSatTrackbar_Scroll(object sender, EventArgs e)
        {
            SatLabel.Text = "Sat.: " + LowSatTrackbar.Value.ToString() + "," + HighSatTrackbar.Value.ToString();
            lowSat = (float)LowSatTrackbar.Value / 100;
        }
        private void HighSatTrackbar_Scroll(object sender, EventArgs e)
        {
            SatLabel.Text = "Sat.: " + LowSatTrackbar.Value.ToString() + "," + HighSatTrackbar.Value.ToString();
            highSat = (float)HighSatTrackbar.Value / 100;
        }
        private void LowLumTrackbar_Scroll(object sender, EventArgs e)
        {
            LumLabel.Text = "Lum. " + LowLumTrackbar.Value.ToString() + "," + HighLumTrackbar.Value.ToString();
            lowLum = (float)LowLumTrackbar.Value / 100;
        }
        private void HighLumTrackbar_Scroll(object sender, EventArgs e)
        {
            LumLabel.Text = "Lum. " + LowLumTrackbar.Value.ToString() + "," + HighLumTrackbar.Value.ToString();
            highLum = (float)HighLumTrackbar.Value / 100;
        }

        // Set hsl filter on when the hsl checkbox is checked
        private void hslCheckBox_CheckedChanged(object sender, EventArgs e)
        {
            hslOn = !hslOn;
        }

        /**
         * Refresh Data timer constantly updates the command interface with values from 
         * different parts of the code whether it be vision tracking or motor speed. It 
         * will also be in charge of logging data into the data logger.
         **/
        private void RefreshData_Tick(object sender, EventArgs e)
        {
            // TODO 
            if (sensors != true)
            {
                for (int i = 0; i < ifKit.sensors.Count; i++)
                {
                    ifKit.sensors[i].Sensitivity = 0;
                }
                sensors = true;
            }

            // Keep track of time for logger and update data every second
            dataRefreshCount++;

            // Updates variables every second and increases the time elapsed in the code
            if (dataRefreshCount == 10)
            {
                dataRefreshSecond++;
                dataRefreshCount = 0;

                // Sets the time passed using the variables dataRefreshSecond and dataRefreshMinute
                if (dataRefreshMinute < 10 && dataRefreshSecond < 10)
                {
                    time = "0" + dataRefreshMinute + ":0" + dataRefreshSecond + "\t";
                }
                else if (dataRefreshMinute < 10 && dataRefreshSecond >= 10)
                {
                    time = "0" + dataRefreshMinute + ":" + dataRefreshSecond + "\t";
                }
                else if (dataRefreshMinute >= 10 && dataRefreshSecond >= 10)
                {
                    time = dataRefreshMinute + ":" + dataRefreshSecond + "\t";
                }
            }

            // Updates variables to adjust the time
            if (dataRefreshSecond == 60)
            {
                dataRefreshMinute++;
                dataRefreshSecond = 0;
            }

            // Display circle tracking information
            FCircleCenterLabel.Text = "Circle Center Point: " + FCircleCenter.X + ", " + FCircleCenter.Y;
            FCircleRadiusLabel.Text = "Circle Radius: " + FCircleRadius;
            BCircleCenterLabel.Text = "Circle Center Point: " + BCircleCenter.X + ", " + BCircleCenter.Y;
            BCircleRadiusLabel.Text = "Circle Radius: " + BCircleRadius;
            // Display rectangle tracking information
            
            BRectCenterLabel.Text = "Rectangle Center Point: " + rectCenterX + ", " + rectCenterY;
            // Display quadrilateral tracking information
            FPointLabel1.Text = "Quadrilateral Point 1: " + pFOne.X + ", " + pFOne.Y;
            FPointLabel2.Text = "Quadrilateral Point 2: " + pFTwo.X + ", " + pFTwo.Y;
            FPointLabel3.Text = "Quadrilateral Point 3: " + pFThree.X + ", " + pFThree.Y;
            FPointLabel4.Text = "Quadrilateral Point 4: " + pFFour.X + ", " + pFFour.Y;
            BPointLabel1.Text = "Quadrilateral Point 1: " + pBOne.X + ", " + pBOne.Y;
            BPointLabel2.Text = "Quadrilateral Point 2: " + pBTwo.X + ", " + pBTwo.Y;
            BPointLabel3.Text = "Quadrilateral Point 3: " + pBThree.X + ", " + pBThree.Y;
            BPointLabel4.Text = "Quadrilateral Point 4: " + pBFour.X + ", " + pBFour.Y;
            if (videoProcessing.rects != null)
            {
                FRectCenterLabel.Text = videoProcessing.rects.Length.ToString();
            }
            // Display Motor Values
            frontLeftMotorLabel.Text = "Front Left Motor: " + -fLMotorControl.motors[0].Velocity;
            frontRightMotorLabel.Text = "Front Right Motor: " + -fRMotorControl.motors[0].Velocity;
            backLeftMotorLabel.Text = "Back Left Motor: " + -bLMotorControl.motors[0].Velocity;
            backRightMotorLabel.Text = "Back Right Motor: " + -bRMotorControl.motors[0].Velocity;
            rightVerticalMotorLabel.Text = "Right Vertical Motor: " + rVMotorControl.motors[0].Velocity;
            leftVerticalMotorLabel.Text = "Left Vertical Motor: " + lVMotorControl.motors[0].Velocity;
            // Display Depth
            auvDepthLabel.Text = "Depth: " + Math.Round(((Convert.ToDouble(textBox4.Text) - 92) / 2.1), 2) + "ft";
            depthLabel2.Text = "Depth: " + Math.Round((Convert.ToDouble(textBox4.Text)), 2);
            auvDepthDisplay.Location = new System.Drawing.Point(1, 12 + (15 * ((Convert.ToInt32(textBox4.Text)) - 93)));
            // Display Voltage
            voltageLabel.Text = "Voltage: " + Math.Round(((Convert.ToInt32(textBox2.Text) / 13.62) - 36.7107), 2).ToString();
            // Display Current
            currentLabel.Text = "Current: " + Math.Round(((Convert.ToInt32(textBox3.Text) / 13.2) - 37.8787), 2).ToString();
        }


        //IfKit attach event handler
        //Here we'll display the interface kit details as well as determine how many output and input
        //fields to display as well as determine the range of values for input sensitivity slider
        void ifKit_Attach(object sender, AttachEventArgs e)
        {
            InterfaceKit ifKit = (InterfaceKit)sender;
            ifKit.sensors[1].Sensitivity = 100;
            int i;
            for (i = 0; i < ifKit.inputs.Count; i++)
            {
                digiInArray[i].Visible = true;
                digiInArray[i].ForeColor = Color.Wheat;
                ((Label)digitalInputsGroupBox.Controls["digitalInputLabel" + i.ToString()]).Visible = true;
            }

            for (i = 0; i < ifKit.outputs.Count; i++)
            {
                digiOutArray[i].Visible = true;
                digiOutArray[i].Checked = ifKit.outputs[i];
                digiOutArray[i].Enabled = true;
                digiOutDispArray[i].Visible = true;

                ((Label)digitalOutputsGroupBox.Controls["digitalOutputLabel" + i.ToString()]).Visible = true;
            }

            if (ifKit.sensors.Count > 0)
            {
                for (i = 0; i < ifKit.sensors.Count; i++)
                {
                    sensorInArray[i].Visible = true;
                    ((Label)analogInputsGroupBox.Controls["analogInputLabel" + i.ToString()]).Visible = true;
                    if (ifKit.ID == Phidget.PhidgetID.INTERFACEKIT_2_2_2
                        || ifKit.ID == Phidget.PhidgetID.INTERFACEKIT_8_8_8
                        || ifKit.ID == Phidget.PhidgetID.INTERFACEKIT_8_8_8_w_LCD)
                        ifKit.sensors[i].DataRate = 32;
                }

                label8.Visible = true;
                sensitivityTxt.Visible = true;
                inputTrk.Visible = true;
                ratioChk.Visible = true;

                // Blacklist
                if ((ifKit.Name != "Phidget Touch Slider") && (ifKit.Name != "Phidget Touch Rotation"))
                    sensorsButton.Visible = true;
                else
                    sensorsButton.Visible = false;
            }
            else
            {
                label8.Visible = false;
                sensitivityTxt.Visible = false;
                inputTrk.Visible = false;
                ratioChk.Visible = false;
            }

            inputTrk.Enabled = true;
            try
            {
                if (ifKit.sensors.Count > 0)
                    inputTrk.Value = 0;

                sensitivityTxt.Text = inputTrk.Value.ToString();
            }
            catch { }

            try
            {
                ratioChk.Enabled = true;
                ratioChk.Checked = ifKit.ratiometric;
            }
            catch (PhidgetException pe)
            {
                if (pe.Type == PhidgetException.ErrorType.PHIDGET_ERR_UNSUPPORTED)
                {
                    ratioChk.Enabled = false;
                    ratioChk.Visible = false;
                }
            }
        }

        //Ifkit detach event handler
        //Here we display the attached status, which will be false as the device is disconnected. 
        //We will also clear the display fields and hide the inputs and outputs.
        void ifKit_Detach(object sender, DetachEventArgs e)
        {
            InterfaceKit ifKit = (InterfaceKit)sender;

            int i;
            for (i = 0; i < 16; i++)
            {
                digiInArray[i].Visible = false;
                digiInArray[i].Checked = false;
                ((Label)digitalInputsGroupBox.Controls["digitalInputLabel" + i.ToString()]).Visible = false;
            }
            for (i = 0; i < 16; i++)
            {
                digiOutArray[i].Enabled = false;
                digiOutArray[i].Visible = false;
                digiOutDispArray[i].Visible = false;

                ((Label)digitalOutputsGroupBox.Controls["digitalOutputLabel" + i.ToString()]).Visible = false;
                digiOutDispArray[i].Checked = false;
            }
            for (i = 0; i < 8; i++)
            {
                sensorInArray[i].Visible = false;
                sensorInArray[i].Text = "";
                ((Label)analogInputsGroupBox.Controls["analogInputLabel" + i.ToString()]).Visible = false;
            }

            label8.Visible = false;
            sensitivityTxt.Visible = false;
            inputTrk.Visible = false;
            inputTrk.Value = 0;
            inputTrk.Enabled = false;
            sensitivityTxt.Text = "";

            ratioChk.Enabled = false;
            ratioChk.Checked = false;
            ratioChk.Visible = false;
            sensorsButton.Visible = false;
            if ((advSensorForm != null) && (advSensorForm.IsDisposed == false))
                advSensorForm.Close();
        }

        //Digital input change event handler
        //Here we check or uncheck the corresponding input checkbox based 
        //on the index of the digital input that generated the event
        void ifKit_InputChange(object sender, InputChangeEventArgs e)
        {
            digiInArray[e.Index].Checked = e.Value;
        }

        //Sensor input change event handler
        //Set the textbox content based on the input index that is communicating
        //with the interface kit
        void ifKit_SensorChange(object sender, SensorChangeEventArgs e)
        {
            sensorInArray[e.Index].Text = e.Value.ToString();

            if (advSensorForm != null)
                advSensorForm.SetValue(e.Index, e.Value);
        }

        private void ratioChk_CheckedChanged(object sender, EventArgs e)
        {
            try
            {
                ifKit.ratiometric = ratioChk.Checked;
            }
            catch (PhidgetException) { }
        }


        //This method creates the digital input array that corresponds to the group of checkboxes
        //we are using to represent the state of the digital inputs as reported on the interface kit.
        //It will also initialize the visibiity of the check boxes and labeling to false.
        private void makeDigiInArray()
        {
            for (int i = 0; i < 16; i++)
            {
                ((Label)digitalInputsGroupBox.Controls["digitalInputLabel" + i.ToString()]).Visible = false;
                digiInArray[i] = (CheckBox)digitalInputsGroupBox.Controls["checkBox" + (i + 1).ToString()];
                digiInArray[i].Visible = false;
            }
        }

        //This method will associate the digiOutArray with check boxes that control the state of the
        //digital outputs on the interface kit. It will also associate the digiOutDispArray with check
        //boxes that represent the state of the outputs as reported on the interfacekit, and initialize 
        //the visibility of the check boxes and labeling to false.
        private void makeDigiOutArray()
        {
            for (int i = 0; i < 16; i++)
            {
                ((Label)digitalOutputsGroupBox.Controls["digitalOutputLabel" + i.ToString()]).Visible = false;
                digiOutArray[i] = (CheckBox)digitalOutputsGroupBox.Controls["checkBox" + (i + 17).ToString()];
                digiOutArray[i].Visible = false;
                digiOutDispArray[i] = (CheckBox)digitalOutputsGroupBox.Controls["checkBoxReport" + i.ToString()];
                digiOutDispArray[i].Visible = false;
            }
        }

        //This method will associate the sensorInArray with text boxes that display the current values 
        //of the analog inputs on the interface kit. It will also initialize the visibility of the
        //text boxes and analog input labeling to false.
        private void makeSensorInArray()
        {
            for (int i = 0; i < 8; i++)
            {
                ((Label)analogInputsGroupBox.Controls["analogInputLabel" + i.ToString()]).Visible = false;
                sensorInArray[i] = (TextBox)analogInputsGroupBox.Controls["textBox" + (i + 1).ToString()];
                sensorInArray[i].Visible = false;
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if ((advSensorForm == null) || (advSensorForm.IsDisposed))
            {
                advSensorForm = new InterfaceKit_full.AdvancedSensorForm();
                advSensorForm.Show();
                advSensorForm.SetSensorCount(ifKit.sensors.Count);
                for (int i = 0; i < ifKit.sensors.Count; i++)
                {
                    advSensorForm.SetValue(i, Convert.ToInt32(sensorInArray[i].Text));
                }
            }
            advSensorForm.Focus();
        }
        public enum DataMode { Text, Hex }
        public enum LogMsgType { Incoming, Outgoing, Normal, Warning, Error };
        // The main control for communicating through the RS-232 port
        private SerialPort comport = new SerialPort();

        // Various colors for logging info
        private Color[] LogMsgTypeColor = { Color.Blue, Color.Green, Color.Black, Color.Orange, Color.Red };

        // Temp holder for whether a key was pressed
        private bool KeyHandled = false;


        private string[] OrderedPortNames()
        {
            // Just a placeholder for a successful parsing of a string to an integer
            int num;

            // Order the serial port names in numberic order (if possible)
            return SerialPort.GetPortNames().OrderBy(a => a.Length > 3 && int.TryParse(a.Substring(3), out num) ? num : 0).ToArray();
        }


        Boolean Set = false;

        /*
         * Code used to set the velocity of all the motors in a more simpler and less space intensive way. 
         */
        private void setMotorVelocity(double fl, double fr, double bl, double br, double rv, double lv)
        {
            fRMotorControl.motors[0].Velocity = limit100(fr * Convert.ToDouble(fLMotorOffset.Text));
            fLMotorControl.motors[0].Velocity = limit100(fl * Convert.ToDouble(fRMotorOffset.Text));
            bRMotorControl.motors[0].Velocity = limit100(-br * Convert.ToDouble(bLMotorOffset.Text));
            bLMotorControl.motors[0].Velocity = limit100(-bl * Convert.ToDouble(bRMotorOffset.Text));
            rVMotorControl.motors[0].Velocity = limit100(-rv * Convert.ToDouble(rVMotorOffset.Text));
            lVMotorControl.motors[0].Velocity = limit100(lv * Convert.ToDouble(lVMotorOffset.Text));
        }

        private void setMoveVelocity(double x, double y, double z, double turn)
        {
            setMotorVelocity(-turn+y+x, turn+y-x, -turn+y-x, turn+y+x, z, -z);
        }

        private double limit100(double input)
        {
            double temp = input > 100 ? 100 : input;
            temp = temp < -100 ? -100 : temp;
            return temp;
        }

        /*
         * Code Used to test the motors. All horizontal motors should be going forward and all vertical ones shouls be going up.
         */
        private Boolean flag = false;
        private void motorTestButton_Click_1(object sender, EventArgs e)
        {
            
        }

        SlimDX.DirectInput.Joystick joystick;
        JoystickState state = new JoystickState();

        void BootGamePort()
        {
            // make sure that DirectInput has been initialized
            DirectInput dinput = new DirectInput();

            // search for devices
            foreach (DeviceInstance device in dinput.GetDevices(DeviceClass.GameController, DeviceEnumerationFlags.AttachedOnly))
            {
                // create the device
                try
                {
                    joystick = new SlimDX.DirectInput.Joystick(dinput, device.InstanceGuid);
                    break;
                }
                catch (DirectInputException)
                {
                }
            }

            if (joystick == null)
            {
                // are a fail message to appear on screen if no game pad present
                return;
            }

            foreach (DeviceObjectInstance deviceObject in joystick.GetObjects())
            {
                if ((deviceObject.ObjectType & ObjectDeviceType.Axis) != 0)
                    joystick.GetObjectPropertiesById((int)deviceObject.ObjectType).SetRange(-100, 100);
            }

            // acquire the device.
            joystick.Acquire();
        }

        private void JoystickTimer_Tick(object sender, EventArgs e)
        {
            if (joystick.Acquire().IsFailure)
                return;

            if (joystick.Poll().IsFailure)
                return;

            state = joystick.GetCurrentState();
            if (Result.Last.IsFailure)
                return;

            if (state.X != 0)
            {
                // Code for Left and Right Movement.
                setMotorVelocity(state.X, -state.X, -state.X, state.X, -state.RotationZ, -state.RotationZ);
            }

            double motorFowardLeft = state.X;

            // Code for Foward and Back movements.
            if (motorFowardLeft == 0)
            {
                setMotorVelocity(-state.Y, -state.Y, -state.Y, -state.Y, -state.RotationZ, -state.RotationZ);
            }

            //Declares the Joystick Values into X and Y
            double ValuePosX = state.X;
            double ValuePosY = -state.Y;

            // Code for Foward Right Direction.
            if (ValuePosY > 0 && ValuePosX > 0)
            {
                setMotorVelocity(state.X, 0, 0, -state.Y, -state.RotationZ, -state.RotationZ);
            }

            // Code for Fowrad Left Direction.
            if (ValuePosY > 0 && ValuePosX < 0)
            {
                setMotorVelocity(0, -state.Y, -state.X, 0, -state.RotationZ, -state.RotationZ);
            }

            // Code for Back Left Direction.
            if (ValuePosY < 0 && ValuePosX < 0)
            {
                setMotorVelocity(state.Y, 0, 0, state.X, -state.RotationZ, -state.RotationZ);
            }

            // Code for Back Right Direction.
            if (ValuePosY < 0 && ValuePosX > 0)
            {
                setMotorVelocity(0, -state.Y, -state.X, 0, -state.RotationZ, -state.RotationZ);
            }

            // Code to get output from buttons.
            bool[] buttons = state.GetButtons();
            String strText = "";
            for (int b = 0; b < buttons.Length; b++)
            {

                if (buttons[b])
                    strText += b.ToString("00 ", CultureInfo.CurrentCulture);

                // Code for left turning.
                if (buttons[4])
                {
                    setMotorVelocity(-20, 20, -20, 20, -state.RotationZ, -state.RotationZ);
                }
                //Code for right turning.
                if (buttons[5])
                {
                    setMotorVelocity(20, -20, 20, -20, -state.RotationZ, -state.RotationZ);
                }
            }
        }

        private void missionPlannerPlayButton_Click(object sender, EventArgs e)
        {
            MissionPlanner.Enabled = true;
            hslCheckBox.Checked = true;
            lineCount = Convert.ToInt32(LineCountDefault.Text);
            timeL = DateTime.Now;
            int secondsPassed = Convert.ToInt32(secondsPassedTextBox.Text);
        }

        private void missionPlannerStopButton_Click(object sender, EventArgs e)
        {
            MissionPlanner.Enabled = false;
            setMotorVelocity(0, 0, 0, 0, 0, 0);
            hslCheckBox.Checked = false;
            buoyNumber = true;
            mission1.Checked = true;
            // Reset all vision tracking variables
            FNeutralButton.Checked = true;
            BNeutralButton.Checked = true;
            slopeCount = 0;
            ramReady = false;
            locked = false;
            gyroReady = false;
            ramDone = false;
            rectTestX = rectCenterX;
            rectTestY = rectCenterY;
            videoProcessing.rectInView = false;
            videoProcessing.slopeInView = false;
            // Reset HSL
            LowHueTrackbar.Value = 0;
            HighHueTrackbar.Value = 400;
            LowSatTrackbar.Value = 0;
            HighSatTrackbar.Value = 100;
            LowLumTrackbar.Value = 0;
            HighLumTrackbar.Value = 100;
            LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            lineCount = 1;
            // Stop Teleoperated Controls
            JoystickTimer.Enabled = false;
            floodFillBox.Checked = false;
            // Reset Gyro
            gyroHeading[0] = 0;
            gyroHeading[1] = 0;
            gyroHeading[2] = 0;

            minWidth = 70;
            TeleOpTimer.Enabled = false;
        }

        /*
         * The mission planner which is used to organize and make all the decisions of the code in Project AUV. It could be considered its AI system
         * as it tells the computer what to do and when as well as what missions we can use. 
         * 
         * Mission 1: Locks heading of gyro and set depth for gate height. Start bottom vision tracking for orange rectangle and move forward until rectangle(path marker) is detected.
         * Mission 2: Center the rectangle detected with the center of the screen.
         * Mission 3: Start slope tracking orange rectangle(path marker) until Slope is infinite or close to that.
         * Mission 4: Go to buoy depth and start foward scan until buoy(rectangle) is detected.
         * Mission 5: Track the buoy until it is in the center of the screen and proceed to ram it. 
         * Mission 6: If second buoy is selected back up and and drift sideways until buoy(rectangle) is detected.
         * Mission 7: Track the buoy until it is in the center of the screen and proceed to ram it. 
         * Mission 8: Go to depth above buoys and go forward until orange rectangle is detected
         * Mission 9: Center the rectangle detected with the center of the screen.
         * Mission 10: Start slope tracking orange rectangle(path marker) until Slope is infinite or close to that.
         */

        Boolean buoyNumber = true;
        // Mission Code
        private void MissionPlanner_Tick(object sender, EventArgs e)
        { 
            double speed = Convert.ToDouble(missionSpeed.Text);
            double minSpeed = Convert.ToDouble(missionMinSpeed.Text);
            int depthVar = Convert.ToInt32(missionDepthVar.Text);
            int headingVar = Convert.ToInt32(missionHeadingVar.Text);
            int depth = Convert.ToInt32(textBox4.Text);
            double depthSpeed = Convert.ToDouble(DepthSpeed.Text);
            double maxSpeed = Convert.ToDouble(MaxSpeed.Text);
            double forwardSpeed = Convert.ToDouble(ForwardSpeed.Text);
            double heading = -gyroHeading[2];//traxHeading;
            double x = 0, y = 0, z = 0, turn = 0;
            //label102.Text = FrontCameraNormal.Image.Size.Width.ToString();
            secondCount++;
            if (secondCount == 100)
            {
                seconds++;
                secondCount = 0;
            }

            if (!checkBox1.Checked)
            {
                // Reset Gyro
                setMotorVelocity(0, 0, 0, 0, 0, 0);
                buoyNumber = true;
                mission1.Checked = true;
                // Reset all vision tracking variables
                FNeutralButton.Checked = true;
                BNeutralButton.Checked = true;
                slopeCount = 0;
                ramReady = false;
                locked = false;
                gyroReady = false;
                ramDone = false;
                rectTestX = rectCenterX;
                rectTestY = rectCenterY;
                videoProcessing.rectInView = false;
                videoProcessing.slopeInView = false;
                lineCount = 1;
                // Stop Teleoperated Controls
                JoystickTimer.Enabled = false;
                floodFillBox.Checked = false;
                // Reset Gyro
                gyroHeading[0] = 0;
                gyroHeading[1] = 0;
                gyroHeading[2] = 0;

                minWidth = 70;
                TeleOpTimer.Enabled = false;
                gyroHeading[0] = 0;
                gyroHeading[1] = 0;
                gyroHeading[2] = 0;
                timeL = DateTime.Now;
                videoProcessing.slopeInView = false;
            }

            if (checkBox1.Checked)
            {
                if (mission1.Checked)
                {
                    hslCheckBox.Checked = true;
                    // Turn On Rectangle Tracking
                    setColor("line");
                    BSlopeButton.Checked = true;

                    // Basic Motor Control Variables
                    int targetDepth = Convert.ToInt32(mission1Depth.Text);
                    int targetHeading = 0;

                    ///////////////////////////////////////////////////////////
                    turn = pidHeading.PIDupdateAngle(heading, 0);
                    z = pidDepth.PIDupdate(depth, targetDepth);
                    ///////////////////////////////////////////////////////////
                    if (depth <= targetDepth + depthVar && depth >= targetDepth - depthVar && heading <= targetHeading + headingVar && heading >= targetHeading - headingVar && !videoProcessing.slopeInView)
                    {
                        y =  forwardSpeed;
                    }
                    else
                    {
                        y = 0;
                    }
                    
                    // Set Vision Counter to see if rectangle is detected
                    if (videoProcessing.slopeInView)
                    {
                        if (rectCount < Convert.ToDouble(rectIdentifyTime.Text))
                        {
                            rectCount++;
                        }
                        else
                        {
                            mission2.Checked = true;
                            seconds = 0;
                            secondCount = 0;
                            rectCount = 0;
                            
                        }
                    }
                    else
                    {
                        rectCenterX = 0;
                        rectCenterY = 0;
                        rectCount = 0;
                    }

                    if (seconds > Convert.ToInt32(mission1Timeout.Text))
                    {
                        MissionPlanner.Enabled = false;
                        timeout();
                    }

                    speedLabel.Text = "Speed: " + speed;
                    rotationSpeedLabel.Text = "Rotation Speed: " + turn;
                }
                else if (mission2.Checked)
                {
                    mission3.Checked = true;
                    timeL = DateTime.Now;
                }
                else if (mission3.Checked)
                {
                    double xTrackVar = Math.Abs(Convert.ToInt32(trackingVariabillity.Text));
                    double yTrackVar = Math.Abs(Convert.ToInt32(trackingVariabillity.Text) * 2);
                    // Turn On Slope Tracking
                    setColor("line");
                    BSlopeButton.Checked = true;
                    double slope = videoProcessing.slope;
                    slopeLabel.Text = slope.ToString();

                    turn = pidSlope.PIDupdateAngle(-slope, 0);
                    x =  pidX.PIDupdate(-rectCenterX, 0);
                    y =  pidY.PIDupdate(rectCenterY, 0);

                    if (inRange(slope, -4, 4) && inRange(rectCenterX, -xTrackVar, xTrackVar) && inRange(rectCenterY, -yTrackVar, yTrackVar) && Math.Abs(rectCenterX) > 0.0001)
                    {
                        if ( DateTime.Now.Subtract(timeL).TotalMilliseconds > 1000)//slopeCount >= 100)
                        {
                            if (lineCount == 1)
                            {
                                // Stop all motors
                                setMotorVelocity(0, 0, 0, 0, 0, 0);
                                lineCount = 2;
                                // Reset Gyro
                                gyroHeading[0] = 0;
                                gyroHeading[1] = 0;
                                gyroHeading[2] = 0;

                                rectTestX = rectCenterX;
                                rectTestY = rectCenterY;
                                rectCount = 0;

                                BNeutralButton.Checked = true;
                                // Go to mission 4
                                mission4.Checked = true;
                                // Decide which buoy to track
                                if (buoy1Red.Checked)
                                {
                                    //setColor("redbuoy");
                                }
                                else if (buoy1Green.Checked)
                                {
                                    setColor("greenbuoy");
                                }
                                else if (buoy1Yellow.Checked)
                                {
                                    setColor("yellowbuoy");
                                }
                            }
                            else if (lineCount == 2)
                            {
                                lineCount = 3;
                                // Stop all motors
                                setMotorVelocity(0, 0, 0, 0, 0, 0);
                                timeL = DateTime.Now;
                                // Reset Gyro
                                gyroHeading[0] = 0;
                                gyroHeading[1] = 0;
                                gyroHeading[2] = 0;

                                rectTestX = rectCenterX;
                                rectTestY = rectCenterY;
                                rectCount = 0;
                                mission8.Checked = true;
                                BNeutralButton.Checked = true;
                            }
                            else if (lineCount == 3)
                            {
                                // Stop all motors
                                setMotorVelocity(0, 0, 0, 0, 0, 0);
                                timeL = DateTime.Now;
                                // Reset Gyro
                                gyroHeading[0] = 0;
                                gyroHeading[1] = 0;
                                gyroHeading[2] = 0;

                                rectTestX = rectCenterX;
                                rectTestY = rectCenterY;
                                rectCount = 0;
                                mission9.Checked = true;
                                BNeutralButton.Checked = true;
                            }
                        }
                        else
                        {
                            // Stop all motors
                            setMotorVelocity(0, 0, 0, 0, 0, 0);
                            slopeCount++;
                        }
                    }
                    else
                    {
                        timeL = DateTime.Now;
                        slopeCount = 0;
                    }
                }
                else if (mission4.Checked)
                {
                    double littleTarget = Convert.ToDouble(littleTargetTextBox.Text);
                    double bigTarget = Convert.ToDouble(bigTargetTextBox.Text);

                    setColor("redbuoy");
                    if (hslRadioButton.Checked)
                    {
                        hslCheckBox.Checked = true;
                        floodFillBox.Checked = false;
                        
                    }

                    if (floodFillRadioButton.Checked)
                    {
                        floodFillBox.Checked = true;
                        hslCheckBox.Checked = false;
                    }


                    // Turn On Rectangle Tracking
                    FRectangleButton.Checked = true;

                    // Basic Motor Control Variables
                    int targetDepth = Convert.ToInt32(buoyTargetDepth.Text);
                    double buoyTargetHeading = Convert.ToDouble(targetBuoyDepthTextBox.Text);

                    int targetHeading = 0;
                    double xTrackVar = Convert.ToDouble(trackingVariabillity.Text);
                    double yTrackVar = Convert.ToDouble(trackingVariabillity.Text) * .75;

                    heading = -heading;
                    double rotationSpeed = (speed / 100) * (heading * 1.8);

                    if (rotationSpeed < minSpeed)
                    {
                        rotationSpeed = minSpeed;
                    }
                    else if (rotationSpeed > maxSpeed)
                    {
                        rotationSpeed = maxSpeed;
                    }
                    label67.Text = FRectangle.Width.ToString();
                    if (FRectangle.Width < 40)
                    {
                        pidFCam.kp = littleTarget; //little
                    }
                    else
                    {
                        pidFCam.kp = bigTarget; //big
                    }
                    
                    // Set Vision Counter to see if rectangle is detected
                    if (videoProcessing.rectInView == true)// != 0 && rectCenterY != 0 && rectCenterX != rectTestX && rectCenterY != rectTestY)
                    {
                        turn = pidFCam.PIDupdate((rectCenterX - 320)/300.0*10, 0);
                        y = 50;//forwardSpeed;
                        z = pidDepth.PIDupdate(-(rectCenterY - 240) / 75.0, 0);

                        if (FRectangle.Width > Convert.ToInt32(ramRadius.Text))// && rectCenterX < 320 + xTrackVar && rectCenterX > 320 - xTrackVar && rectCenterY < 240 + yTrackVar && rectCenterY > 240 - yTrackVar)
                        {
                            y = 0;
                            if (rectCenterX < 320 + xTrackVar && rectCenterX > 320 - xTrackVar && rectCenterY < 240 + yTrackVar && rectCenterY > 240 - yTrackVar)
                            {
                                if (rectCount < 25)
                                {
                                    setMotorVelocity(0, 0, 0, 0, 0, 0);
                                    rectCount++;
                                }
                                else
                                {
                                    setMotorVelocity(0, 0, 0, 0, 0, 0);
                                    mission5.Checked = true;
                                    rectCount = 0;
                                    // Reset Gyro
                                    gyroHeading[0] = 0;
                                    gyroHeading[1] = 0;
                                    gyroHeading[2] = 0;
                                    timeL = DateTime.Now;
                                    label103.Text = "Ram Procedure Initiated";
                                    hslCheckBox.Checked = true;
                                    floodFillBox.Checked = false;
                                }
                            }
                            else
                            {
                                rectCount = 0;
                            }
                        }
                        else
                        {
                            rectCount = 0;
                        }
                    }
                    else
                    {
                        turn = pidHeading.PIDupdateAngle(-heading, buoyTargetHeading);
                        if (depth <= targetDepth + depthVar && depth >= targetDepth - depthVar && heading <= targetHeading + headingVar && heading >= targetHeading - headingVar)
                        {
                            y = forwardSpeed;
                        }
                        else
                        {
                            z = pidDepth.PIDupdate(depth, targetDepth);
                            y = 0;
                        }
                    }

                    speedLabel.Text = "Speed: " + speed;
                    rotationSpeedLabel.Text = "Rotation Speed: " + rotationSpeed;
                }
                else if (mission5.Checked)
                {
                    // Ram Buoy
                    turn = pidHeading.PIDupdateAngle(heading, 0);
                    DateTime t = DateTime.Now;

                    if (t.Subtract(timeL).TotalMilliseconds < Convert.ToDouble(ramForwardTime.Text))
                    {
                        y = 50;
                    }
                    else if (t.Subtract(timeL).TotalMilliseconds < Convert.ToDouble(ramBackwardsTime.Text))
                    {
                        y = -50;
                    }
                    else
                    {
                        y = 0;
                        if (buoyNumber)
                        {
                            mission6.Checked = true;
                            // Reset Gyro
                            
                            buoyNumber = false;
                            timeL = DateTime.Now;
                        }
                        else
                        {
                            //mission7.Checked = true;
                        }
                    }
                }
                else if (mission6.Checked)
                {
                    // Go over and past buoys
                    FNeutralButton.Checked = true;
                    DateTime t = DateTime.Now;

                    if (t.Subtract(timeL).TotalMilliseconds < Convert.ToDouble(PassTime.Text))
                    {
                        y = 50;
                    }
                    else if (t.Subtract(timeL).TotalMilliseconds < leftTime)
                    {
                        x = direction;
                    }
                    else
                    {
                        mission7.Checked = true;
                        timeL = DateTime.Now;
                        y = 0;
                    }
                    z = pidDepth.PIDupdate(depth, 110);
                    turn = pidHeading.PIDupdate(heading, 0);
                }
                else if (mission7.Checked)
                {
                    // Go Past buoys
                    DateTime t = DateTime.Now;

                    if (t.Subtract(timeL).TotalMilliseconds < 500)
                    {
                        z = pidDepth.PIDupdate(depth, 120);
                        turn = pidHeading.PIDupdate(heading, 0);
                        y = 50;
                    }
                    else
                    {
                        mission1.Checked = true;
                        
                    }
                }
                else if (mission8.Checked)
                {
                    double gateHeading = Convert.ToDouble(gateHeadingTextBox.Text);
                    double parallelParkingAngle = Convert.ToDouble(parallelParkingTextBox.Text);
                    double forwardBeforeTurn = Convert.ToDouble(forwardBeforeTurnTextBox.Text);
                    double sidewaysTime = Convert.ToDouble(sidewaysTimeTextBox.Text);
                    double goSideways = Convert.ToDouble(goSidewaysTextBox.Text);
                    double turnBackToNormal = Convert.ToDouble(turnBackToNormalTextBox.Text);
                    double rotation = Convert.ToDouble(rotationTextBox.Text);
                    
                    // Pass through gate
                    DateTime t = DateTime.Now;
                    z = pidDepth.PIDupdate(depth, 120);
                     //gate heading
                    if (t.Subtract(timeL).TotalMilliseconds < forwardBeforeTurn) // Time to go forward before turning
                    {
                        y = 50;
                    }
                    else if (t.Subtract(timeL).TotalMilliseconds < sidewaysTime) // Time For Turning
                    {
                        gateHeading = gateHeading - rotation;
                        x = 0;
                    }
                    else if (t.Subtract(timeL).TotalMilliseconds < goSideways) // Go Sideways 
                    {
                        gateHeading = gateHeading - rotation;
                        x = -50;
                    }
                    else if (t.Subtract(timeL).TotalMilliseconds < turnBackToNormal) // Turn Back To Normal
                    {
                        x = 0;
                    }
                    else
                    {
                        //start searching for two lines
                        // Turn On Rectangle Tracking
                        setColor("line");
                        BSlopeButton.Checked = true;

                        // Basic Motor Control Variables
                        int targetDepth = Convert.ToInt32(mission1Depth.Text);
                        int targetHeading = 0;

                        ///////////////////////////////////////////////////////////
                        turn = pidHeading.PIDupdateAngle(heading, 0);
                        z = pidDepth.PIDupdate(depth, targetDepth);
                        ///////////////////////////////////////////////////////////
                        if (depth <= targetDepth + depthVar && depth >= targetDepth - depthVar && heading <= targetHeading + headingVar && heading >= targetHeading - headingVar && !videoProcessing.slopeInView)
                        {
                            y = forwardSpeed;
                        }
                        else
                        {
                            y = 0;
                        }

                        // Set Vision Counter to see if rectangle is detected
                        if (videoProcessing.slopeInView)
                        {
                            if (rectCount < Convert.ToDouble(rectIdentifyTime.Text))
                            {
                                rectCount++;
                            }
                            else
                            {
                                mission10.Checked = true;
                                timeL = DateTime.Now;
                                seconds = 0;
                                secondCount = 0;
                                rectCount = 0;
                            }
                        }
                        else
                        {
                            rectCenterX = 0;
                            rectCenterY = 0;
                            rectCount = 0;
                        }
                        speedLabel.Text = "Speed: " + speed;
                        rotationSpeedLabel.Text = "Rotation Speed: " + turn;
                    }

                    turn = pidHeading.PIDupdate(heading, gateHeading);
                }
                else if (mission9.Checked)
                {
                    // Track double lines
                    double xTrackVar = Math.Abs(Convert.ToInt32(trackingVariabillity.Text));
                    double yTrackVar = Math.Abs(Convert.ToInt32(trackingVariabillity.Text) * 2);
                    // Turn On Slope Tracking
                    setColor("line");
                    BSlopeButton.Checked = true;
                    double slope = videoProcessing.slope;
                    slopeLabel.Text = slope.ToString();
                    if (videoProcessing.blobs != null)
                    {
                        if (videoProcessing.blobs.Length == 1)
                        {
                            turn = pidSlope.PIDupdateAngle(-videoProcessing.slope1, 0);
                            x = pidX.PIDupdate(-(videoProcessing.slopeCenterX1 - 320), 0);
                            y = pidY.PIDupdate(videoProcessing.slopeCenterY1 - 240, 0);
                        }
                        else if (videoProcessing.blobs.Length == 2)
                        {
                            double greaterSlope;
                            double lesserSlope;
                            if (videoProcessing.slope1 > videoProcessing.slope2)
                            {
                                greaterSlope = videoProcessing.slope1;
                                lesserSlope = videoProcessing.slope2;
                            }
                            else
                            {
                                greaterSlope = videoProcessing.slope2;
                                lesserSlope = videoProcessing.slope1;
                            }

                            double calcSlope = ((greaterSlope + lesserSlope) / 2) - ((greaterSlope - lesserSlope) / 2);
                            double midX = (videoProcessing.slopeCenterX1 + videoProcessing.slopeCenterX2 - 640) / 2;
                            double midY = (videoProcessing.slopeCenterY1 + videoProcessing.slopeCenterY2 - 480) / 2;

                            turn = pidSlope.PIDupdateAngle(-calcSlope, 0);
                            x = pidX.PIDupdate(-(midX), 0);
                            y = pidY.PIDupdate(midY, 0);

                            if (inRange(calcSlope, -4, 4) && inRange(midX, -xTrackVar, xTrackVar) && inRange(midY, -yTrackVar, yTrackVar) && Math.Abs(midX) > 0.0001)
                            {
                                if (slopeCount >= 50) //(DateTime.Now.Subtract(timeL).TotalMilliseconds > 1000)
                                {
                                    // Stop all motors
                                    setMotorVelocity(0, 0, 0, 0, 0, 0);
                                    timeL = DateTime.Now;
                                    // Reset Gyro
                                    gyroHeading[0] = 0;
                                    gyroHeading[1] = 0;
                                    gyroHeading[2] = 0;

                                    rectTestX = rectCenterX;
                                    rectTestY = rectCenterY;
                                    rectCount = 0;
                                    mission10.Checked = true;
                                    BNeutralButton.Checked = true;
                                }
                                else
                                {
                                    // Stop all motors
                                    setMotorVelocity(0, 0, 0, 0, 0, 0);
                                    slopeCount++;
                                }
                            }
                            else
                            {
                                timeL = DateTime.Now;
                                slopeCount = 0;
                            }
                        }
                    }
                }
                if (mission10.Checked)
                {
                    double binOffSet = Convert.ToDouble(offSetTextBox.Text);
                    double octOffSet = Convert.ToDouble(binOffSetTextBox.Text);
                    // Go forward and try to drop markers with time
                    DateTime t = DateTime.Now;

                    if (t.Subtract(timeL).TotalMilliseconds < 92000)
                    {
                        z = pidDepth.PIDupdate(depth, 105);
                        turn = pidHeading.PIDupdate(heading, octOffSet);
                        y = 50;
                    }
                    //else if (t.Subtract(timeL).TotalMilliseconds < (74000))
                    //{
                    //    z = pidDepth.PIDupdate(depth, 105);
                    //    turn = pidHeading.PIDupdate(heading, octOffSet);
                    //    y = 50;
                    //    //checkBox81.Checked = true;
                    //    //checkBox82.Checked = true;
                    //}
                    //else if (t.Subtract(timeL).TotalMilliseconds < 96740)
                    //{
                    //    z = pidDepth.PIDupdate(depth, 105);
                    //    turn = pidHeading.PIDupdate(heading, 17);
                    //    y = 50;
                    //    
                    //}
                    else
                    {
                        z = pidDepth.PIDupdate(depth, 95);
                        turn = pidHeading.PIDupdate(heading, 0);
                        y = 0;
                    }
                }
                
                setMoveVelocity(x, y, z, turn);
            }
            else
            {
                setMoveVelocity(0, 0, 0, 0);
            }
        }

        int lineCount = 2;
        public bool inRange(double val, double low, double high)
        {
            bool temp = true;
            if (val > high) temp = false;
            if (val < low) temp = false;
            return temp;
        }

        /*
         * If robot gets lost during the mission it will time out causing it to surface. 
         */
        private void timeout()
        {
            MissionPlanner.Enabled = false;
            setMotorVelocity(0, 0, 0, 0, 100, 100);
        }

        private void MotorIndicatorTimer_Tick(object sender, EventArgs e)
        {
            double fLMotor = fLMotorControl.motors[0].Velocity;
            double fRMotor = fRMotorControl.motors[0].Velocity;
            double bLMotor = -bLMotorControl.motors[0].Velocity;
            double bRMotor = -bRMotorControl.motors[0].Velocity;
            double rVMotor = rVMotorControl.motors[0].Velocity;
            double lVMotor = lVMotorControl.motors[0].Velocity;

            if (fLMotor == 0 && fRMotor == 0 && bLMotor == 0 && bRMotor == 0)
            {
                // Neutral
                auvDirection.Image = System.Drawing.Image.FromFile("C:/Users/Falcon Robotics/Downloads/AUV 2012 Top Shot.jpg");
                auvDirection.Location = new System.Drawing.Point(36, 29);
            }
            else if (fLMotor > 0 && fRMotor > 0 && bLMotor > 0 && bRMotor > 0)
            {
                // Go Forwards
                auvDirection.Image = System.Drawing.Image.FromFile("C:/Users/Falcon Robotics/Downloads/AUV 2012 Top Shot.jpg");
                auvDirection.Location = new System.Drawing.Point(36, 4);
            }
            else if (fLMotor < 0 && fRMotor < 0 && bLMotor < 0 && bRMotor < 0)
            {
                // Go Backwards
                auvDirection.Image = System.Drawing.Image.FromFile("C:/Users/Falcon Robotics/Downloads/AUV 2012 Top Shot.jpg");
                auvDirection.Location = new System.Drawing.Point(36, 54);
            }
            else if (fLMotor > 0 && fRMotor < 0 && bLMotor > 0 && bRMotor < 0)
            {
                // Rotate Right
                auvDirection.Image = System.Drawing.Image.FromFile("C:/Users/Falcon Robotics/Downloads/AUV 2012 Top Shot Rotated Right.jpg");
                auvDirection.Location = new System.Drawing.Point(36, 29);
            }
            else if (fLMotor < 0 && fRMotor > 0 && bLMotor < 0 && bRMotor > 0)
            {
                // Rotate Left
                auvDirection.Image = System.Drawing.Image.FromFile("C:/Users/Falcon Robotics/Downloads/AUV 2012 Top Shot Rotated Left.jpg");
                auvDirection.Location = new System.Drawing.Point(36, 29);
            }
            else if (fLMotor > 0 && fRMotor < 0 && bLMotor < 0 && bRMotor > 0)
            {
                // Go Right
                auvDirection.Image = System.Drawing.Image.FromFile("C:/Users/Falcon Robotics/Downloads/AUV 2012 Top Shot.jpg");
                auvDirection.Location = new System.Drawing.Point(61, 29);
            }
            else if (fLMotor < 0 && fRMotor > 0 && bLMotor > 0 && bRMotor < 0)
            {
                // Go Left
                auvDirection.Image = System.Drawing.Image.FromFile("C:/Users/Falcon Robotics/Downloads/AUV 2012 Top Shot.jpg");
                auvDirection.Location = new System.Drawing.Point(11, 29);
            }
        }

        private void Trackbar_ValueChanged(object sender, EventArgs e)
        {
            lowHue = LowHueTrackbar.Value;
            highHue = HighHueTrackbar.Value;
            HueLabel.Text = "Hue: " + LowHueTrackbar.Value.ToString() + "," + HighHueTrackbar.Value.ToString();

            lowSat = (float)LowSatTrackbar.Value / 100;
            highSat = (float)HighSatTrackbar.Value / 100;
            SatLabel.Text = "Sat: " + LowSatTrackbar.Value.ToString() + "," + HighSatTrackbar.Value.ToString();

            lowLum = (float)LowLumTrackbar.Value / 100;
            highLum = (float)HighLumTrackbar.Value / 100;
            LumLabel.Text = "Lum: " + LowLumTrackbar.Value.ToString() + "," + HighLumTrackbar.Value.ToString();
        }

        private void SetColorButtonsClicked(object sender, EventArgs e)
        {
            if (sender == setLineColor)
            {
                System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Colors\linecolor.txt");
                String text = file.ReadLine();

                int count2 = 0;
                int[] array = new int[8];

                for (int count = 0; count <= text.Length - 1; count++)
                {
                    if (text.Substring(count, 1).Equals(","))
                    {
                        array[count2] = count;
                        count2++;
                    }
                }

                LowHueTrackbar.Value = Convert.ToInt32(text.Substring(0, array[0]));
                HighHueTrackbar.Value = Convert.ToInt32(text.Substring(array[0] + 1, array[1] - array[0] - 1));
                LowSatTrackbar.Value = Convert.ToInt32(text.Substring(array[1] + 1, array[2] - array[1] - 1));
                HighSatTrackbar.Value = Convert.ToInt32(text.Substring(array[2] + 1, array[3] - array[2] - 1));
                LowLumTrackbar.Value = Convert.ToInt32(text.Substring(array[3] + 1, array[4] - array[3] - 1));
                HighLumTrackbar.Value = Convert.ToInt32(text.Substring(array[4] + 1, text.Length - array[4] - 1));
                LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            }
            else if (sender == setRedBuoyColor)
            {
                System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Colors\redbuoy.txt");
                String text = file.ReadLine();

                int count2 = 0;
                int[] array = new int[8];

                for (int count = 0; count <= text.Length - 1; count++)
                {
                    if (text.Substring(count, 1).Equals(","))
                    {
                        array[count2] = count;
                        count2++;
                    }
                }

                LowHueTrackbar.Value = Convert.ToInt32(text.Substring(0, array[0]));
                HighHueTrackbar.Value = Convert.ToInt32(text.Substring(array[0] + 1, array[1] - array[0] - 1));
                LowSatTrackbar.Value = Convert.ToInt32(text.Substring(array[1] + 1, array[2] - array[1] - 1));
                HighSatTrackbar.Value = Convert.ToInt32(text.Substring(array[2] + 1, array[3] - array[2] - 1));
                LowLumTrackbar.Value = Convert.ToInt32(text.Substring(array[3] + 1, array[4] - array[3] - 1));
                HighLumTrackbar.Value = Convert.ToInt32(text.Substring(array[4] + 1, text.Length - array[4] - 1));
                LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            }
            else if (sender == setGreenBuoyColor)
            {
                System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Colors\greenbuoy.txt");
                String text = file.ReadLine();

                int count2 = 0;
                int[] array = new int[8];

                for (int count = 0; count <= text.Length - 1; count++)
                {
                    if (text.Substring(count, 1).Equals(","))
                    {
                        array[count2] = count;
                        count2++;
                    }
                }

                LowHueTrackbar.Value = Convert.ToInt32(text.Substring(0, array[0]));
                HighHueTrackbar.Value = Convert.ToInt32(text.Substring(array[0] + 1, array[1] - array[0] - 1));
                LowSatTrackbar.Value = Convert.ToInt32(text.Substring(array[1] + 1, array[2] - array[1] - 1));
                HighSatTrackbar.Value = Convert.ToInt32(text.Substring(array[2] + 1, array[3] - array[2] - 1));
                LowLumTrackbar.Value = Convert.ToInt32(text.Substring(array[3] + 1, array[4] - array[3] - 1));
                HighLumTrackbar.Value = Convert.ToInt32(text.Substring(array[4] + 1, text.Length - array[4] - 1));
                LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            }
            else if (sender == setYellowBuoyColor)
            {
                System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Colors\yellowBuoy.txt");
                String text = file.ReadLine();

                int count2 = 0;
                int[] array = new int[8];

                for (int count = 0; count <= text.Length - 1; count++)
                {
                    if (text.Substring(count, 1).Equals(","))
                    {
                        array[count2] = count;
                        count2++;
                    }
                }

                LowHueTrackbar.Value = Convert.ToInt32(text.Substring(0, array[0]));
                HighHueTrackbar.Value = Convert.ToInt32(text.Substring(array[0] + 1, array[1] - array[0] - 1));
                LowSatTrackbar.Value = Convert.ToInt32(text.Substring(array[1] + 1, array[2] - array[1] - 1));
                HighSatTrackbar.Value = Convert.ToInt32(text.Substring(array[2] + 1, array[3] - array[2] - 1));
                LowLumTrackbar.Value = Convert.ToInt32(text.Substring(array[3] + 1, array[4] - array[3] - 1));
                HighLumTrackbar.Value = Convert.ToInt32(text.Substring(array[4] + 1, text.Length - array[4] - 1));
                LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            }
            else if (sender == setObstacleColor)
            {
                System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Colors\obstacle.txt");
                String text = file.ReadLine();

                int count2 = 0;
                int[] array = new int[8];

                for (int count = 0; count <= text.Length - 1; count++)
                {
                    if (text.Substring(count, 1).Equals(","))
                    {
                        array[count2] = count;
                        count2++;
                    }
                }

                LowHueTrackbar.Value = Convert.ToInt32(text.Substring(0, array[0]));
                HighHueTrackbar.Value = Convert.ToInt32(text.Substring(array[0] + 1, array[1] - array[0] - 1));
                LowSatTrackbar.Value = Convert.ToInt32(text.Substring(array[1] + 1, array[2] - array[1] - 1));
                HighSatTrackbar.Value = Convert.ToInt32(text.Substring(array[2] + 1, array[3] - array[2] - 1));
                LowLumTrackbar.Value = Convert.ToInt32(text.Substring(array[3] + 1, array[4] - array[3] - 1));
                HighLumTrackbar.Value = Convert.ToInt32(text.Substring(array[4] + 1, text.Length - array[4] - 1));
                LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            }
        }

        private void SaveButtonColorsClicked(object sender, EventArgs e)
        {
            if (sender == saveLineColor)
            {
                String text = LowHueTrackbar.Value.ToString() + "," + HighHueTrackbar.Value.ToString()
              + "," + LowSatTrackbar.Value.ToString() + "," + HighSatTrackbar.Value.ToString()
              + "," + LowLumTrackbar.Value.ToString() + "," + HighLumTrackbar.Value.ToString();

                System.IO.File.WriteAllText(@"C:\Colors\linecolor.txt", text);
            }
            else if (sender == saveRedBuoyColor)
            {
                String text = LowHueTrackbar.Value.ToString() + "," + HighHueTrackbar.Value.ToString()
              + "," + LowSatTrackbar.Value.ToString() + "," + HighSatTrackbar.Value.ToString()
              + "," + LowLumTrackbar.Value.ToString() + "," + HighLumTrackbar.Value.ToString();

                System.IO.File.WriteAllText(@"C:\Colors\redbuoy.txt", text);
            }
            else if (sender == saveGreenBuoyColor)
            {
                String text = LowHueTrackbar.Value.ToString() + "," + HighHueTrackbar.Value.ToString()
              + "," + LowSatTrackbar.Value.ToString() + "," + HighSatTrackbar.Value.ToString()
              + "," + LowLumTrackbar.Value.ToString() + "," + HighLumTrackbar.Value.ToString();

                System.IO.File.WriteAllText(@"C:\Colors\greenbuoy.txt", text);
            }
            else if (sender == saveYellowBuoyColor)
            {
                String text = LowHueTrackbar.Value.ToString() + "," + HighHueTrackbar.Value.ToString()
              + "," + LowSatTrackbar.Value.ToString() + "," + HighSatTrackbar.Value.ToString()
              + "," + LowLumTrackbar.Value.ToString() + "," + HighLumTrackbar.Value.ToString();

                System.IO.File.WriteAllText(@"C:\Colors\yellowbuoy.txt", text);
            }
            else if (sender == saveObstacleColor)
            {
                String text = LowHueTrackbar.Value.ToString() + "," + HighHueTrackbar.Value.ToString()
              + "," + LowSatTrackbar.Value.ToString() + "," + HighSatTrackbar.Value.ToString()
              + "," + LowLumTrackbar.Value.ToString() + "," + HighLumTrackbar.Value.ToString();

                System.IO.File.WriteAllText(@"C:\Colors\obstacle.txt", text);
            }
        }

        private void setColor(String color)
        {
            if (color.Equals("redbuoy"))
            {
                System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Colors\redbuoy.txt");
                String text = file.ReadLine();

                int count2 = 0;
                int[] array = new int[8];

                for (int count = 0; count <= text.Length - 1; count++)
                {
                    if (text.Substring(count, 1).Equals(","))
                    {
                        array[count2] = count;
                        count2++;
                    }
                }

                LowHueTrackbar.Value = Convert.ToInt32(text.Substring(0, array[0]));
                HighHueTrackbar.Value = Convert.ToInt32(text.Substring(array[0] + 1, array[1] - array[0] - 1));
                LowSatTrackbar.Value = Convert.ToInt32(text.Substring(array[1] + 1, array[2] - array[1] - 1));
                HighSatTrackbar.Value = Convert.ToInt32(text.Substring(array[2] + 1, array[3] - array[2] - 1));
                LowLumTrackbar.Value = Convert.ToInt32(text.Substring(array[3] + 1, array[4] - array[3] - 1));
                HighLumTrackbar.Value = Convert.ToInt32(text.Substring(array[4] + 1, text.Length - array[4] - 1));
                LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            }
            else if (color.Equals("greenbuoy"))
            {
                System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Colors\greenbuoy.txt");
                String text = file.ReadLine();

                int count2 = 0;
                int[] array = new int[8];

                for (int count = 0; count <= text.Length - 1; count++)
                {
                    if (text.Substring(count, 1).Equals(","))
                    {
                        array[count2] = count;
                        count2++;
                    }
                }

                LowHueTrackbar.Value = Convert.ToInt32(text.Substring(0, array[0]));
                HighHueTrackbar.Value = Convert.ToInt32(text.Substring(array[0] + 1, array[1] - array[0] - 1));
                LowSatTrackbar.Value = Convert.ToInt32(text.Substring(array[1] + 1, array[2] - array[1] - 1));
                HighSatTrackbar.Value = Convert.ToInt32(text.Substring(array[2] + 1, array[3] - array[2] - 1));
                LowLumTrackbar.Value = Convert.ToInt32(text.Substring(array[3] + 1, array[4] - array[3] - 1));
                HighLumTrackbar.Value = Convert.ToInt32(text.Substring(array[4] + 1, text.Length - array[4] - 1));
                LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            }
            else if (color.Equals("yellowbuoy"))
            {
                System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Colors\yellowbuoy.txt");
                String text = file.ReadLine();

                int count2 = 0;
                int[] array = new int[8];

                for (int count = 0; count <= text.Length - 1; count++)
                {
                    if (text.Substring(count, 1).Equals(","))
                    {
                        array[count2] = count;
                        count2++;
                    }
                }

                LowHueTrackbar.Value = Convert.ToInt32(text.Substring(0, array[0]));
                HighHueTrackbar.Value = Convert.ToInt32(text.Substring(array[0] + 1, array[1] - array[0] - 1));
                LowSatTrackbar.Value = Convert.ToInt32(text.Substring(array[1] + 1, array[2] - array[1] - 1));
                HighSatTrackbar.Value = Convert.ToInt32(text.Substring(array[2] + 1, array[3] - array[2] - 1));
                LowLumTrackbar.Value = Convert.ToInt32(text.Substring(array[3] + 1, array[4] - array[3] - 1));
                HighLumTrackbar.Value = Convert.ToInt32(text.Substring(array[4] + 1, text.Length - array[4] - 1));
                LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            }
            else if (color.Equals("line"))
            {
                System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Colors\linecolor.txt");
                String text = file.ReadLine();

                int count2 = 0;
                int[] array = new int[8];

                for (int count = 0; count <= text.Length - 1; count++)
                {
                    if (text.Substring(count, 1).Equals(","))
                    {
                        array[count2] = count;
                        count2++;
                    }
                }

                LowHueTrackbar.Value = Convert.ToInt32(text.Substring(0, array[0]));
                HighHueTrackbar.Value = Convert.ToInt32(text.Substring(array[0] + 1, array[1] - array[0] - 1));
                LowSatTrackbar.Value = Convert.ToInt32(text.Substring(array[1] + 1, array[2] - array[1] - 1));
                HighSatTrackbar.Value = Convert.ToInt32(text.Substring(array[2] + 1, array[3] - array[2] - 1));
                LowLumTrackbar.Value = Convert.ToInt32(text.Substring(array[3] + 1, array[4] - array[3] - 1));
                HighLumTrackbar.Value = Convert.ToInt32(text.Substring(array[4] + 1, text.Length - array[4] - 1));
                LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            }
            else if (color.Equals("obstacle"))
            {
                System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Colors\obstacle.txt");
                String text = file.ReadLine();

                int count2 = 0;
                int[] array = new int[8];

                for (int count = 0; count <= text.Length - 1; count++)
                {
                    if (text.Substring(count, 1).Equals(","))
                    {
                        array[count2] = count;
                        count2++;
                    }
                }

                LowHueTrackbar.Value = Convert.ToInt32(text.Substring(0, array[0]));
                HighHueTrackbar.Value = Convert.ToInt32(text.Substring(array[0] + 1, array[1] - array[0] - 1));
                LowSatTrackbar.Value = Convert.ToInt32(text.Substring(array[1] + 1, array[2] - array[1] - 1));
                HighSatTrackbar.Value = Convert.ToInt32(text.Substring(array[2] + 1, array[3] - array[2] - 1));
                LowLumTrackbar.Value = Convert.ToInt32(text.Substring(array[3] + 1, array[4] - array[3] - 1));
                HighLumTrackbar.Value = Convert.ToInt32(text.Substring(array[4] + 1, text.Length - array[4] - 1));
                LowHueTrackbar.ValueChanged += new System.EventHandler(Trackbar_ValueChanged);
            }
        }

        private void button1_Click_1(object sender, EventArgs e)
        {

        }

        private void dataGridView1_CellContentClick(object sender, DataGridViewCellEventArgs e)
        {

        }

        private void read_Click(object sender, EventArgs e)
        {
            dataGridView1.Rows[0].Cells[1].Value = pidHeading.kp;
            dataGridView1.Rows[1].Cells[1].Value = pidHeading.kd;
            dataGridView1.Rows[2].Cells[1].Value = pidDepth.kp;
            dataGridView1.Rows[3].Cells[1].Value = pidDepth.kd;
            dataGridView1.Rows[4].Cells[1].Value = pidDepth.PIDgetKi();
            dataGridView1.Rows[5].Cells[1].Value = pidX.kp;
            dataGridView1.Rows[6].Cells[1].Value = pidX.kd;
            dataGridView1.Rows[7].Cells[1].Value = pidSlope.kp;
            dataGridView1.Rows[8].Cells[1].Value = pidSlope.kd;
            dataGridView1.Rows[9].Cells[1].Value = pidFCam.kp;
            dataGridView1.Rows[10].Cells[1].Value = pidFCam.kd;
        }

        private void set_Click(object sender, EventArgs e)
        {
            string temp;
            temp = dataGridView1.Rows[0].Cells[1].Value.ToString();
            for (int i = 1; i < 11; i++)
            {
                temp = temp + "," + dataGridView1.Rows[i].Cells[1].Value.ToString();
            }
            Project_AUV_2015.Properties.Settings.Default.PIDvals = temp;
            Project_AUV_2015.Properties.Settings.Default.Save();
            pidHeading.kp = Convert.ToDouble(dataGridView1.Rows[0].Cells[1].Value);
            pidHeading.kd = Convert.ToDouble(dataGridView1.Rows[1].Cells[1].Value);
            pidDepth.kp = Convert.ToDouble(dataGridView1.Rows[2].Cells[1].Value);
            pidDepth.kd = Convert.ToDouble(dataGridView1.Rows[3].Cells[1].Value);
            pidDepth.PIDsetKi(Convert.ToDouble(dataGridView1.Rows[4].Cells[1].Value));
            pidX.kp = pidY.kp = Convert.ToDouble(dataGridView1.Rows[5].Cells[1].Value);
            pidX.kd = pidY.kd = Convert.ToDouble(dataGridView1.Rows[6].Cells[1].Value);
            pidSlope.kp = Convert.ToDouble(dataGridView1.Rows[7].Cells[1].Value);
            pidSlope.kd = Convert.ToDouble(dataGridView1.Rows[8].Cells[1].Value);
            pidFCam.kp = Convert.ToDouble(dataGridView1.Rows[9].Cells[1].Value);
            pidFCam.kd = Convert.ToDouble(dataGridView1.Rows[10].Cells[1].Value);
            // test
        }
        
        private void targetDepth_Scroll(object sender, EventArgs e)
        {

        }

        private void motorTestButton_KeyDown(object sender, System.Windows.Forms.KeyEventArgs e)
        {
            TeleOpTimer.Enabled = true;
            double teleSpeed = TeleopSpeed.Value;

            if (e.KeyCode == Keys.W)
            {
                //setMotorVelocity(teleSpeed, teleSpeed, teleSpeed, teleSpeed, -telePIDDepth, telePIDDepth); 
                setMotorVelocity(80, 72, 80, 72, 0, 0);
            }
            if (e.KeyCode == Keys.A)
            {
                setMotorVelocity(-teleSpeed, teleSpeed, teleSpeed, -teleSpeed, -telePIDDepth, telePIDDepth); 
            }
            if (e.KeyCode == Keys.S)
            {
                setMotorVelocity(-teleSpeed, -teleSpeed, -teleSpeed, -teleSpeed, -telePIDDepth, telePIDDepth); 
            }
            if (e.KeyCode == Keys.D)
            {
                setMotorVelocity(teleSpeed, -teleSpeed, -teleSpeed, teleSpeed, -telePIDDepth, telePIDDepth); 
            }
            if (e.KeyCode == Keys.R)
            {
                //teleTargetDepth--;
                //setMotorVelocity(0, 0, 0, 0, -teleSpeed, teleSpeed); 
            }
            if (e.KeyCode == Keys.F)
            {
                //teleTargetDepth++;
                //setMotorVelocity(0, 0, 0, 0, teleSpeed, -teleSpeed); 
            }
            if (e.KeyCode == Keys.Q)
            {
                setMotorVelocity(-20, 20, -20, 20, -telePIDDepth, telePIDDepth);
            }
            if (e.KeyCode == Keys.E)
            {
                setMotorVelocity(20, -20, 20, -20, -telePIDDepth, telePIDDepth);
            }
        }

        private void motorTestButton_KeyUp(object sender, System.Windows.Forms.KeyEventArgs e)
        {
            setMotorVelocity(0, 0, 0, 0, -telePIDDepth, telePIDDepth); 
        }

        public int teleTargetDepth = 105;
        double telePIDDepth = 0;

        private void TeleOpTimer_Tick(object sender, EventArgs e)
        {
            double slope = videoProcessing.slope;
            label102.Text = slope.ToString();
            int depth = Convert.ToInt32(textBox4.Text);
            label71.Text = teleTargetDepth.ToString();
            telePIDDepth = -pidDepth.PIDupdate(depth, teleTargetDepth);
            lVMotorControl.motors[0].Velocity = limit100(telePIDDepth * Convert.ToDouble(lVMotorOffset.Text));
            rVMotorControl.motors[0].Velocity = limit100(telePIDDepth * Convert.ToDouble(rVMotorOffset.Text));
        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
            teleTargetDepth = trackBar1.Value;
        }

        private void changeToleranceButton_Click(object sender, EventArgs e)
        {
            pictureNumber = Convert.ToInt32(buoy.Text);
            flood1Num1 = Convert.ToInt32(floodFill1Num1.Text);
            flood1Num2 = Convert.ToInt32(floodFill1Num2.Text);
            flood1Num3 = Convert.ToInt32(floodFill1Num3.Text);
            flood2Num1 = Convert.ToInt32(floodFill2Num1.Text);
            flood2Num2 = Convert.ToInt32(floodFill2Num2.Text);
            flood2Num3 = Convert.ToInt32(floodFill2Num3.Text);

            flood3Num1 = Convert.ToInt32(floodFill3Num1.Text);
            flood3Num2 = Convert.ToInt32(floodFill3Num2.Text);
            flood3Num3 = Convert.ToInt32(floodFill3Num3.Text);
            flood4Num1 = Convert.ToInt32(floodFill4Num1.Text);
            flood4Num2 = Convert.ToInt32(floodFill4Num2.Text);
            flood4Num3 = Convert.ToInt32(floodFill4Num3.Text);

        }

        private void button2_Click(object sender, EventArgs e)
        {
            direction = Convert.ToDouble(directionTextBox.Text);
            leftTime = Convert.ToDouble(timeTextBox.Text);
        }

        private void labelRefresh_Tick(object sender, EventArgs e)
        {
            label117.Text = FrontCameraNormal.Height.ToString();
        }
    }
}
 