using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using XBOW440;
using NGCP.IO;
using System.Threading;

namespace UUVMAIN
{

    class ThrustManager
    {
        private static Thread backgroundThrustManager;

        IMU440 navigation;
        PositionTracker position;

        //micro will have references to sonar, temperature, and direction(heading/yaw).
        Microcontroller uCon;

        log lg;

        PID ptch, rll, surge, sway, depth, yaw;

        public double desiredPitch;
        public double desiredRoll;
        public double desiredDepth;
        public double desiredYaw;
        public double desiredSurge;
        public double desiredSway;

        //thrusts
        public double Tp;//pitch
        public double Tr;//roll
        public double Ty;//yaw
        public double Td;//depth
        public double Tsu;//surge
        public double Tsw;//sway

        public pidConf Ppitch;
        public pidConf Proll;
        public pidConf Pyaw;
        public pidConf Pdepth;
        public pidConf Psurge;
        public pidConf Psway;

        private double getDesiredPitch()
        {
            return desiredPitch;
        }
        private double getDesiredRoll()
        {
            return desiredRoll;
        }
        private double getActualPitch()
        {
            return -navigation.pitchAngle * (180 / Math.PI);
        }
        private double getActualRoll()
        {
            return navigation.rollAngle * (180 / Math.PI);
        }
        private void computeTp(double n)
        {
            Tp = (Math.Abs(n) > 5) ? n : 0;
        }
        private void computeTr(double n)
        {
            Tr = (Math.Abs(n) > 5) ? n : 0;
        }

        private double getDesiredYaw()
        {
            return desiredYaw;
        }
        private double getDesiredDepth()
        {
            return (uCon.gamepad_depth - 81 >= 0) ? desiredDepth : -Microcontroller.map(uCon.gamepad_depth, 0, 81, 2, 0);
        }
        private double getActualYaw()
        {
            return uCon.Yaw; //in degrees
        }
        private double getActualDepth()
        {
            return uCon.Depth;//in meters
        }
        private void computeTy(double n)
        {
            Ty = uCon.gamepad_yaw;
        }
        private void computeTd(double n)
        {
            //Td = n;
            //or 
            Td =uCon.gamepad_depth;
        }


        private double getDesiredSurge()
        {
            return desiredSurge;
        }
        private double getDesiredSway()
        {
            return desiredSway;
        }
        private double getActualSurge()
        {
            return position.xvel; //testing
            //this might change to postitionTracker reference
        }
        private double getActualSway()
        {
            return position.yvel; //testing
            //this might change to postitionTracker reference
        }
        private void computeTsu(double n)
        {
            Tsu = uCon.gamepad_surge;
        }
        private void computeTsw(double n)
        {
            Tsw = uCon.gamepad_sway;
        }

        public ThrustManager(IMU440 nav, Microcontroller u, PositionTracker p, log loger)
        {
            //Purpose: Constructor for thrustManager
            //Inputs: the IMU object, and the microcontroller object, which each have the serial communication functions within.
            lg = loger;
            navigation = nav;
            position = p;
            uCon = u;
            desiredPitch = 0;
            desiredRoll = 0;
            desiredDepth = -0.5;
            desiredYaw = 0;
            desiredSurge = 0;
            desiredSway = 0;
            Tp = 0;//pitch
            Tr = 0;//roll
            Ty = 0;//yaw
            Td = 0;//depth
            Tsu = 0;//surge
            Tsw = 0;//sway

            uCon.sensorRequest();

            //gots to tune these pids
            ptch = new PID(0, 0, 0, 60.0, -60.0, 40, -40, getActualPitch, getDesiredPitch, computeTp);
            rll = new PID(.5, 0, .4, 60.0, -60.0, 40, -40, getActualRoll, getDesiredRoll, computeTr);
            surge = new PID(0.6, 0, 0, 3, -3, 75, -75, getActualSurge, getDesiredSurge, computeTsu);
            sway = new PID(0.6, 0, 0, 3, -3, 75, -75, getActualSway, getDesiredSway, computeTsw);
            depth = new PID(0.8, 0.1, .8, 0, -9, 75, -75, getActualDepth, getDesiredDepth, computeTd);
            yaw = new PID(0, 0, 0, 180, -180, 40, -40, getActualYaw, getDesiredYaw, computeTy);


            ptch.Enable();
            rll.Enable();
            //commented cause we dont have everything worked out for yaw, and i haven't written something to calculate velocity
            surge.Enable();
            sway.Enable();
            depth.Enable();
            yaw.Enable(); 

            backgroundThrustManager = new Thread(new ThreadStart(levelingTask));
            backgroundThrustManager.IsBackground = true;
            backgroundThrustManager.Start();

        }

        private void levelingTask()
        {
            //Purpose: background task to continuously send updated controller outputs thru serial to motorcontroller, and update PID vals
            //Inputs:
            while (true)
            {
                uCon.sensorRequest();
                if (Microcontroller.PIDupdates.Count > 0)
                {
                    foreach (pidConf d in Microcontroller.PIDupdates)
                    {
                        switch (d.index)
                        {
                            case 0:
                                desiredPitch = d.desval;
                                ptch.updateTerms(d);
                                //Ppitch = d;
                                break;
                            case 1:
                                desiredRoll = d.desval;
                                rll.updateTerms(d);
                                //Proll = d;
                                break;
                            case 2:
                                desiredSurge = d.desval;
                                surge.updateTerms(d);
                                Psurge = d;
                                break;
                            case 3:
                                desiredSway = d.desval;
                                sway.updateTerms(d);
                                Psway = d;
                                break;
                            case 4:
                                desiredDepth = d.desval;
                                depth.updateTerms(d);
                                Pdepth = d;
                                break;
                            case 5:
                                //desiredYaw = d.desval;
                                //yaw.updateTerms(d);
                                Pyaw = d;
                                break;
                        }
                    }//after we've gone through all of the pid updates, clear them
                    Microcontroller.PIDupdates.Clear();
                }

                //there will be some thread.pause in here, so we're not sending tooooo many updates.

                //send the message. check byte implementation in the works.

                //65 is byte val for A, identifying general thrust.
                //66 is the byte val for B, which identifies corner thruster offsets.
                //the thrust is from -81 to 81.

                //first message has surge, sway, depth and yaw. Yaw is applied to surge(as offset) in the microcontroller
                //uCon.Send(new byte[] { 65, (byte)(Tsu + 81), (byte)(Tsw + 81), (byte)(Td + 81), (byte)(Ty + 81), 88 });
                uCon.Send(new byte[] { 65, (byte)(Tsu), (byte)(Tsw ), (byte)(Td), (byte)(Ty ), 88 });

                Thread.Sleep(25);

                //second message has corner motor leveling, applied as offset on microcontroller.
                uCon.Send(new byte[] { 66, (byte)(((Tp + Tr) / 2) + 81), (byte)(((Tp - Tr) / 2) + 81), (byte)(((-Tr - Tp) / 2) + 81), (byte)((((Tr - Tp) / 2)) + 81), 88 });

                Thread.Sleep(25);
            }
        }
    }

}
