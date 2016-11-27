using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace NGCP.IO
{
    class Sonar
    {
        enum ImagenexReturnHeader
        {
            IPX = 0,
            IMX = 252,
            IGX = 500
        };

        Serial main;
        log lg;

        private int _refreshRate;
        private byte[] returnedData;

        //these are parsed from the returnData
        private byte serialStatus;
        private byte rangeSetting;
        private int profileRange;
        private int numberBytesReturned;

        private bool _isPinging;
        private bool _sonarDataReady;

        private ImagenexReturnHeader currentPacketType;

        private List<double> _lastEval;

        //private Thread continuousSonarPing;


        public bool running { get; set; }

        public bool Pinged()
        {
            return _isPinging;
        }
        public bool DataReady()
        {
            return _sonarDataReady;
        }


        void sensorCallBack(byte[] incoming)
        {
            //Purpose: Sensor callback for the sonar: will take packet in and store info
            //Inputs:
            switch (incoming[1])
            {
                case 0x4D:
                    currentPacketType = ImagenexReturnHeader.IMX;
                    break;
                case 0x47:
                    currentPacketType = ImagenexReturnHeader.IGX;
                    break;
                case 0x50:
                    currentPacketType = ImagenexReturnHeader.IPX;
                    break;
            }
            switch (currentPacketType)
            {
                case ImagenexReturnHeader.IMX:
                    //uh, find out what profilerange is?
                    serialStatus = incoming[4];
                    rangeSetting = incoming[7];
                    profileRange = (((incoming[9] & 0x7E) >> 1) << 8) | (((incoming[9] & 0x01) << 7) | (incoming[8] & 0x7F));
                    numberBytesReturned = ((incoming[11] & 0x7E) >> 1) | (((incoming[11] & 0x01) << 7) | (incoming[10] & 0x7F));
                    //use array.copyto to move rest of the bytes(certain number) into returned array
                    break;
                case ImagenexReturnHeader.IGX:
                    serialStatus = incoming[4];
                    rangeSetting = incoming[7];
                    profileRange = (((incoming[9] & 0x7E) >> 1) << 8) | (((incoming[9] & 0x01) << 7) | (incoming[8] & 0x7F));
                    numberBytesReturned = ((incoming[11] & 0x7E) >> 1) | (((incoming[11] & 0x01) << 7) | (incoming[10] & 0x7F));
                    break;
                case ImagenexReturnHeader.IPX:
                    serialStatus = incoming[4];
                    rangeSetting = incoming[7];
                    profileRange = (((incoming[9] & 0x7E) >> 1) << 8) | (((incoming[9] & 0x01) << 7) | (incoming[8] & 0x7F));
                    numberBytesReturned = ((incoming[11] & 0x7E) >> 1) | (((incoming[11] & 0x01) << 7) | (incoming[10] & 0x7F));
                    break;
            }
            _isPinging = false;
            _sonarDataReady = true;
        }

        //not sure how to do this
        public double Evaluate(double range)
        {
            //get the nearest hit based on echostrength 
            byte defaultEchoBound = 100;
            _lastEval = getHitList(defaultEchoBound);
            return _lastEval[0] * range;
        }

        public double Evaluate(byte defaultEchoBound, double range)
        {
            //get the nearest hit based on echostrength 
            _lastEval = getHitList(defaultEchoBound);
            return _lastEval[0]*range;
        }


        public List<double> getHitList(byte lowbound)
        {
            //get the nearest hit based on echostrength 
            List<int> hits = new List<int>();
            List<double> ret = new List<double>();
            if (_sonarDataReady)
            {
                for (int i = 0; i < numberBytesReturned; i++)
                {
                    if (returnedData[i] > lowbound) hits.Add(i);
                }
                foreach (int hit in hits)
                {
                    ret.Add(hit / (double)currentPacketType);//this becomes filled with the proportions of the full range.
                }
                _sonarDataReady = false;
            }
            return ret;
        }

        private void continuousPing()
        {
            //or convert this to timed
            while (running)
            {
                Thread.Sleep(100);
                sendPing();
            }
        }

        private void sendPing()
        {
            //sends a sonar switch data packet
            //decide which type;
            byte[] msg = new byte[27];
            msg[0] = 0xFE;//headers
            msg[1] = 0x44;//headers
            msg[2] = 0x11;//or 0x12, or 0x13. IDK, findout
            msg[3] = 30; //range (5-50){5, 10,20,30,40,50}
            msg[4] = 0;//reserved
            msg[5] = 0;//reserved
            msg[6] = 0x43;
            msg[7] = 0;//reserved
            msg[8] = 20;//0-40 in dbs (increments of 1)
            msg[9] = 0;//reserved
            msg[10] = 20;//absorbtion:default
            msg[11] = 0;//reserved
            msg[12] = 0;//reserved
            msg[13] = 0;//reserved
            msg[14] = 50;//length of pulse in usec
            msg[15] = 5;//minange in meters/10
            msg[16] = 0;//reserved
            msg[17] = 0;//reserved
            msg[18] = 0;//external trigger control
            msg[19] = 25;//or 50, (*10) number of packets
            msg[20] = 0;//reserved
            msg[21] = 0;//reserved
            msg[22] = 0;//IPX or not
            msg[23] = 0;//reserved
            msg[24] = 10;//switch return data delay (*2) msec
            msg[25] = 0;//reserved
            msg[26] = 0xFD;
            main.Send(msg);
            _isPinging = true;
        }

        public Sonar(string port, int rate,log logn, bool debug = false)
        {
            //Purpose: 
            //Inputs:

            //serial init
            main = new Serial(port, 115200);
            lg = logn;
            _refreshRate = rate;
            returnedData = new byte[600];
            main.PackageMode = Serial.PackageModes.UseFunction;
            main.EscapeToken = new byte[]{252};
            //main.FindPackageEnd = (bytes =>
            //{
            //    int offset = 0;
            //    for (int i = 0; i < bytes.Length - 1; i++)
            //    {
            //        if (bytes[i] == 0x49 && bytes[i + 2] == 0x58 && (bytes[i + 1] == 0x4D || bytes[i + 1] == 0x50 || bytes[i + 1] == 0x47) && bytes[i + 3] == 0x11)
            //        {
            //            offset = i;
            //        }
            //    }
            //    if (bytes[offset + 1] == 0x4D && bytes.Length >= offset + 265)
            //    {
            //        if (bytes[offset + 265] == 0xFC)
            //            return offset + 265;
            //    }
            //    else if (bytes[offset + 1] == 0x47 && bytes.Length >= offset + 513)
            //    {
            //        if (bytes[offset + 513] == 0xFC)
            //            return offset + 513;
            //    }
            //    else if (bytes[offset + 1] == 0x50 && bytes.Length >= offset + 13)
            //    {
            //        if (bytes[offset + 13] == 0xFC)
            //            return offset + 13;
            //    }
            //    return -1;
            //});
            main.PackageReceived = sensorCallBack;
            main.Start();
            //continuousSonarPingTask = new Thread(new ThreadStart(continuousPing));
            //add a task starter here to consistenly send ping packets.
        }

    }
}
