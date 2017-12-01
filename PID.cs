using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;

namespace Project_AUV_2015
{
    class PID
    {
        public double kp;
        double ki = 0;
        public double kd;
        double Outmin;
        double Outmax;
        double imax = 10;
        double limitMult;
        double p;
        double i;
        double d;
        double norm;
        double errlast;
        double Outlast;
        long tlast;
        double err;
        long dt;
        Stopwatch sw = new Stopwatch();

        public void PIDinit(double kP, double kI, double kD, double OutmiN, double OutmaX, double imaX, double Norm)
        {
            kp = kP;
            ki = kI;
            kd = kD;
            norm = Norm;
            Outmin = OutmiN;
            Outmax = OutmaX;
            imax = imaX;
            limitMult = 1;
            PIDreset();
            sw.Start();
        }

        public void PIDsetKi(double k)
        {
            if (Math.Abs(ki) < 0.00001)
            {
                ki = k;
            }
            else
            {
                i = i / ki;
                ki = k;
                i = i * ki;
            }
        }

        public double PIDgetKi()
        {
            return ki;
        }

        // Used to get the PID Motor Value
        public double PIDupdate(double feedback, double setpoint)
        {
            double heading = feedback;
            double zero = setpoint;
            
            
            //System.Threading.Thread.Sleep(10);
            // Do something you want to time
            //sw.Stop();

            dt = sw.ElapsedMilliseconds - tlast;
            tlast = sw.ElapsedMilliseconds;
            err = (zero - heading);

            //Calculate d
            if (dt > 0)
            {
                d = (err - errlast) / dt * 1000 * kd * kp * norm;
            }
            else
            {
                d = 0;
            }
                
            //Calculate p
            p = err * kp * norm;

            //Calculate i
            if (Outlast < (Outmax * limitMult - 0.001) && Outlast > (Outmin * limitMult + 0.001))
            {
                i = i + err * dt * ki / 1000 * kp * norm;
            }

            imax = Math.Abs(imax);
            i = Math.Min(i, imax);
            i = Math.Max(i, -imax);
            
            double Out = p + i + d;
            Out = Math.Min(Out, Outmax);
            Out = Math.Max(Out, Outmin);
            errlast = err;
            Outlast = Out;
            return Out;
        }

       // public double returnP()
      //  {
       //     return d;
       // }

        public double PIDupdateAngle(double feedback, double setpoint)
        {
            while (Math.Abs(setpoint - feedback) > 180)
            {
                setpoint += 360 * ((setpoint - feedback) > 0 ? -1 : 1);
            }
            return PIDupdate(feedback, setpoint);
        }

        public void PIDreset()
        {
            Stopwatch sw = new Stopwatch();
            sw.Start();
            // Do something you want to time
            sw.Stop();

            i = 0;
            Outlast = 0;
            errlast = 0;
            tlast = sw.ElapsedMilliseconds;
        }

        public void PIDsetNorm(float normk)
        {
            float oldnorm = (float)norm;
            normk = Math.Abs(normk);
            normk = Math.Min(normk, 2);
            normk = (float)Math.Max(normk, 0.2);
            norm = normk;

            i = i / oldnorm;
            i = i * norm;
        }
    }
}
