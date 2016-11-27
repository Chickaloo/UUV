using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using NGCP.IO;
using XBOW440;

namespace UUVMAIN
{
    class AutonomousBehaviour
    {
        private IMU440 IMU;
        private Microcontroller uC;
        private PositionTracker pos;
        private ThrustManager lev;
        private log ERRORLOG;

        public AutonomousBehaviour(IMU440 IMU1, Microcontroller uC1, PositionTracker pos1, ThrustManager lev1, log ERRORLOG)
        {
            // TODO: Complete member initialization
            this.IMU = IMU1;
            this.uC = uC1;
            this.pos = pos1;
            this.lev = lev1;
            this.ERRORLOG = ERRORLOG;
        }


    }
}
