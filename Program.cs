using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using XBOW440;
using NGCP.IO;

namespace UUVMAIN
{
    class Program
    {
        public static log ERRORLOG;
        public static IMU440 IMU;
        public static Microcontroller uC;
        public static PositionTracker pos;
        public static ThrustManager lev;
        //public static Sonar sonar;
        //public static AutonomousBehaviour auto;

        static void Main(string[] args)
        {
            //we might consider switching the COM port references over to commandline input
            bool is_init = false;
            bool init_imu = false;
            bool init_due = false;
            int _try = 0;
            ERRORLOG = new log();

            Dictionary<string, string> serials = parseSerialLocations();


            ERRORLOG.LOG(" Starting Program", 0);

            while (!is_init && _try < 5)
            {
                try
                {
                    //externals
                    ERRORLOG.LOG("Fuck you jeff", 0);
                    ERRORLOG.LOG("Initialization: " + _try.ToString() + " of 6", 0);
                    if (!init_imu) IMU = new IMU440(serials["IMU"], 57600,ERRORLOG, false);
                    init_imu = true;
                    ERRORLOG.LOG("Initialized IMU", 0);
                    if (!init_due) uC = new Microcontroller(serials["DUE"], 115200,ERRORLOG);
                    init_due = true;
                    ERRORLOG.LOG("Initialized DUE", 0);
                    //sonar will make an appearance soon.
                    //sonar = new Sonar(serials["SONAR"], log);//port

                    //internals
                    pos = new PositionTracker(IMU, uC,ERRORLOG);
                    lev = new ThrustManager(IMU, uC, pos,ERRORLOG);
                    ERRORLOG.LOG("Initialized ThrustManager", 0);
                    //auto = new AutonomousBehaviour(IMU,uC,pos,lev,ERRORLOG);
                    is_init = true;
                }
                catch (Exception e)
                {
                    ERRORLOG.LOG(e.Message, 0);
                    Thread.Sleep(5000);
                    _try++;
                    is_init = false;
                }
            }
            //this is because communications issues:
            for(int l=0;l<15;l++) uC.sensorRequest();
            while (is_init)
            {

            }
            ERRORLOG.LOG("Exiting", 0);
            //while (true)
            //{//testing driver
            //    Console.WriteLine("pitch: " + Math.Round(IMU.pitchAngle*(180/Math.PI)));
            //    Console.WriteLine("roll: " + Math.Round(IMU.rollAngle * (180 /Math.PI)));
            //    Console.WriteLine("tp: " + Math.Round(lev.Tp,4));
            //    Console.WriteLine("tr: " + Math.Round(lev.Tr,4));
            //    Console.WriteLine(IMU.boardTemp);
            //    Thread.Sleep(200);
            //    Console.Clear();
            //}
        }

        private static Dictionary<string, string> parseSerialLocations()
        {
            Dictionary<string, string> retrn = new Dictionary<string, string>();
            using (System.IO.StreamReader readtext = new System.IO.StreamReader("serial.conf"))
            {
                string k;
                while (!readtext.EndOfStream)
                {
                    k = readtext.ReadLine();
                    retrn.Add(k.Substring(0, k.IndexOf(":")), k.Substring(k.IndexOf(":") + 1, k.Length - (k.IndexOf(":") + 1)));
                }

            }
            return retrn;
        }
    }
}
