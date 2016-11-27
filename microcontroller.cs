using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NGCP.IO
{
    public class Microcontroller
    {
        Serial micro;

        log lg;

        private int magX;
        private int magY;
        private int magZ;
        private int tempura;
        private int barReading;

        private static double compassOffset = 14.0;
        private static double pressureOffset = 1023.0;//1013.9?

        public static List<pidConf> PIDupdates = new List<pidConf>();

        public int gamepad_surge;
        public int gamepad_sway;
        public int gamepad_yaw;
        public int gamepad_depth;

        public double Yaw
        {
            get
            {
                /*math here for yaw calc*/
                double returnAngle;
                if (magY > 0)
                {
                    returnAngle = 90.0 - (Math.Atan2(magX, magY) * (180.0 / Math.PI));
                }
                else if (magY < 0)
                {
                    returnAngle = 270.0 - (Math.Atan2(magX, magY) * (180.0 / Math.PI));
                }
                else
                {
                    returnAngle = (magX > 0) ? 0.0 : 180.0;
                }
                return returnAngle + compassOffset;
            }
        }
        public double Depth
        {
            get
            {
                return (((barReading - pressureOffset) / (973525.7588))>0)?-(barReading - pressureOffset) / (973525.7588):0;
            }
        }
        public int Temperature { get { return tempura; } }
        //going to need a bunch of properties/data members for temperature, pressure, etc.
        //also sonar distance?

        void sensorCallBack(byte[] incoming)
        {
            //Purpose: 
            //Inputs:
            try
            {
                if ((incoming[0] == 0x4D || incoming[0] == 0x53 || incoming[0] == 0x44) && incoming.Length>=4)
                {
                    switch (incoming[0])
                    {
                        case 0x4D:
                            //Magnetometer Reading
                            //decipher into heading?
                            magX = incoming[1];
                            magY = incoming[2];
                            magZ = incoming[3];
                            break;
                        case 0x53:
                            //bar pressure and tempura sensor
                            tempura = (int)incoming[1];
                            barReading = Convert.ToInt32(Convert.ToUInt16((incoming[2] << 8) + incoming[3]))/ 10;
                            Console.WriteLine("Bar Reading: " + barReading.ToString());
                            Console.Clear();
                            break;
                        case 0x44:
                            if (incoming[1] <= 5)
                            {
                                PIDupdates.Add(new pidConf(incoming[1], map(incoming[2], 0, 255, 0, 5), map(incoming[3], 0, 255, 0, 5), map(incoming[4], 0, 255, 0, 5), incoming[5] - 127));
                                lg.LOG("New PID configuration: " + string.Format("P: {0} I:{1} D:{2} dval:{3}", PIDupdates[PIDupdates.Count - 1].Pi, PIDupdates[PIDupdates.Count - 1].Ii, PIDupdates[PIDupdates.Count - 1].Di, PIDupdates[PIDupdates.Count - 1].desval), 0);
                            }
                            else if (incoming[1] == 6)
                            {
                                pressureOffset = BitConverter.ToInt16(new byte[] { incoming[2], incoming[3] }, 0) / 10.0;
                                lg.LOG("New depth Pressure Offset: " + pressureOffset.ToString(), 0);
                            }
                            else if (incoming[1] == 65)
                            {
                                gamepad_depth=incoming[4];
                                gamepad_surge=incoming[2];
                                gamepad_sway=incoming[3];
                                gamepad_yaw=incoming[5];
                            }
                            break;

                    }
                }
            }
            catch(Exception e)
            {

            }
            

        }

        public void sensorRequest()
        {
            micro.Send(new byte[] { 82, 88 });
        }

        /// <summary>
        /// Sends bytes to microcontroller through serial
        /// </summary>
        public void Send(byte[] bytes)
        {
            //Purpose: 
            //Inputs:
            micro.Send(bytes);
        }

        public Microcontroller(string port, int baudrate, log loger, bool debug = false)
        {
            //Purpose: 
            //Inputs:

            //serial init
            lg = loger;
            micro = new Serial(port, baudrate);
            micro.PackageMode = Serial.PackageModes.UseEscapeToken;
            micro.EscapeToken = new byte[]{88};
            micro.PackageReceived = sensorCallBack;
            magX = 0;
            magY = 0;
            magZ = 0;
            micro.Start();
        }
        //because C# ain't got no map function
        public static double map(double val, double ind, double inu, double outd, double outu)
        {
            return ((val - ind) * (outu - outd) / (inu - ind)) + outd;
        }
    }
}
