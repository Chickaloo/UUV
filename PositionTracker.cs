using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using XBOW440;
using NGCP.IO;

namespace NGCP.IO
{
     class PositionTracker
    {
        IMU440 navigation;
        Microcontroller mic;

        log lg;

        public double xvel { get; private set; }
        public double yvel { get; private set; }
        
        //this will handle integration and such of/for the imu and heading;
        //basically: integrating x-y accel, and using the heading to change angling.
        //hopefully

        public PositionTracker(IMU440 i, Microcontroller m,log loger)
        {
            lg = loger;
            navigation = i;
            mic = m;
            xvel = 0;
            yvel = 0;
        }
    }
}
