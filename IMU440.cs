using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NGCP.IO;

namespace XBOW440
{
    class IMU440
    {
        byte[] preamble = { 0x55, 0x55 };

        Serial nav;
        log lg;
        private bool debug;

        public double rollDAngle { get; private set; }
        public double pitchDAngle { get; private set; }
        public double yawSAngle { get; private set; }
        public double xRateCorrected { get; private set; }
        public double yRateCorrected { get; private set; }
        public double zRateCorrected { get; private set; }
        public double xAccel { get; private set; }
        public double yAccel { get; private set; }
        public double zAccel { get; private set; }
        public double nVel { get; private set; }
        public double eVel { get; private set; }
        public double dVel { get; private set; }
        public double xDVel { get; private set; }
        public double yDVel { get; private set; }
        public double zDVel { get; private set; }
        public double longitudeGPS { get; private set; }
        public double latitudeGPS { get; private set; }
        public double altitudeGPS { get; private set; }
        public double xRateTemp { get; private set; }
        public double yRateTemp { get; private set; }
        public double zRateTemp { get; private set; }
        public double boardTemp { get; private set; }
        public double timeitow { get; private set; }
        public double bitstatus { get; private set; }
        public double pitchAngle { get { return Math.Atan2(-xAccel, zAccel); } private set { } }//radians
        public double rollAngle { get { return Math.Atan2(-yAccel, Math.Sqrt((xAccel * xAccel) + (zAccel * zAccel))); } private set { } }//radians

        void ping()
        {
            byte[] packet = new byte[4];
            packet[0] = preamble[0];
            packet[1] = preamble[1];
            packet[2] = 0x50;
            packet[3] = 0x4B;
            nav.Send(packet);
        }

        void navCallback(byte[] incoming)
        {
            //first 2 bytes is always 0x5555, second 2 bytes is the packet type
            if (debug)
                Console.WriteLine("Incoming: " + incoming.Length);
            if (preamble[0] == incoming[0] && preamble[1] == incoming[1]) //packet is good
            {
                //get the packet type as a short, switch them for the endian
                short packetType = BitConverter.ToInt16(new byte[] { incoming[3], incoming[2] }, 0);
                if (packetType == 0x504B) //0x504B, 'PK', ping
                {
                    Console.WriteLine("Nav440 - Ping!");
                }
                else
                {
                    //in packets other than ping the 5th byte is the size of the payload in bytes
                    byte size = incoming[4];
                    //the payload is next
                    byte[] payload = new byte[size];
                    for (byte i = 0; i < size; i++)
                    {
                        payload[i] = incoming[5 + i];

                    }
                    //the CRC is the last 2 bytes after the payload
                    byte[] crc = new byte[2];
                    crc[0] = incoming[incoming.Length - 2];
                    crc[1] = incoming[incoming.Length - 1];

                    switch (packetType)
                    {
                        case 0x4348: //0x4348, 'CH', echo
                            Console.Write("Nav440 - Echo: ");
                            for (byte j = 2; j < size; j++)
                            {
                                Console.Write(incoming[j]);
                            }
                            Console.WriteLine();
                            break;
                        case 0x4152: //0x4152, 'AR', Algorithm Reset Response
                            Console.WriteLine("Nav440 - Algorithm Reset");
                            break;
                        case 0x5352: //0x5352, 'SR', Software reset response
                            Console.WriteLine("Nav440 - Software Reset");
                            break;
                        case 0x5743: //0x5743, 'WC', Calibration Acknoledgement
                            Console.Write("Nav440 - Calibration Acknowledged. Type: ");
                            //The payload is the calibration type, always 2 bytes
                            short calibrationType = BitConverter.ToInt16(new byte[] { payload[0], payload[1] }, 0);
                            switch (calibrationType)
                            {
                                case 0x0009: //0x0009, Begin magnetic alignment without automatic termination.
                                    Console.WriteLine("Magnetic calibration without automatic termination, rotate device more than 360 degrees.");
                                    break;
                                case 0x000B: //0x000B, Terminate magnetic alignment
                                    Console.WriteLine("Terminate magnetic alignment.");
                                    break;
                                case 0x000C: //0x000C, Begin magnetic alignment with automatic termination
                                    Console.WriteLine("Begin magnetic alignment with automatic termination, rotate device through 380 degrees.");
                                    break;
                                case 0x000E: //0x000E, Wrote megentic calibration
                                    Console.WriteLine("Wrote magnetic calibration to EEPROM");
                                    break;
                            }
                            break;
                        case 0x4343: //0x4343, 'CC', Calibration completed
                            Console.Write("Nav440 - Calibration Completed. Type: ");
                            //The payload is the calibration type, always 2 bytes
                            calibrationType = BitConverter.ToInt16(new byte[] { payload[0], payload[1] }, 0);
                            switch (calibrationType)
                            {
                                case 0x000B: //0x000B, Terminate magnetic alignment
                                    Console.WriteLine("Terminate magnetic alignment.");
                                    break;
                                case 0x000C: //0x000C, Begin magnetic alignment with automatic termination
                                    Console.WriteLine("Begin magnetic alignment with automatic termination, rotate device through 380 degrees.");
                                    break;
                            }
                            int xHardIron = BitConverter.ToInt16(new byte[] { payload[2], payload[3] }, 0) * (int)(2.0 / Math.Pow(2, 16)); //include scale
                            int yHardIron = BitConverter.ToInt16(new byte[] { payload[4], payload[5] }, 0) * (int)(2.0 / Math.Pow(2, 16));
                            uint softIronScaleRatio = (uint)BitConverter.ToInt16(new byte[] { payload[6], payload[7] }, 0) * (uint)(2.0 / Math.Pow(2, 16));
                            Console.WriteLine("X Hard Iron: " + xHardIron);
                            Console.WriteLine("Y Hard Iron: " + yHardIron);
                            Console.WriteLine("Soft Iron Scale Ratio: " + softIronScaleRatio);
                            break;
                        case 0x1515:
                            Console.Write("Nav440 - Error: Failed input packet type.");
                            break;
                        case 0x4944: //0x4944, 'ID', identification packet
                            uint serialNumber = (uint)BitConverter.ToInt32(new byte[] { payload[0], payload[1], payload[2], payload[3] }, 0);
                            string modelString = "";
                            byte currentbyte = payload[4];
                            int i = 4;
                            while (currentbyte != 0x00)
                            {
                                modelString += currentbyte;
                                i++;
                                currentbyte = payload[i];
                            }
                            Console.WriteLine("Nav440 - Serial Number: " + serialNumber);
                            Console.WriteLine("Nav440 - Model String: " + modelString);
                            break;
                        case 0x5652: //0x5652, 'VR', Version
                            uint major = payload[0];
                            uint minor = payload[1];
                            uint patch = payload[2];
                            uint stage = payload[3];
                            uint buildNumber = payload[4];
                            Console.WriteLine("Nav440 - Version: " + major + "." + minor + "." + patch + "." + stage + "." + buildNumber);
                            break;
                        case 0x5331: //0x5331, 'S1', Scaled Sensor 1 Packet
                            StringBuilder strin = new StringBuilder();
                            strin.Append(BitConverter.ToString(payload));
                            if (debug) Console.WriteLine(strin);
                            //this is the Scaled Sensor 1 packet
                            //it's 24 bits long
                            //then the xRateCorrected in radians per sec, the x angular rate corrected
                            xRateCorrected = BitConverter.ToInt16(new byte[] { payload[7], payload[6] }, 0) * 7.0 * Math.PI / Math.Pow(2, 16);
                            //then yRateCorrected
                            yRateCorrected = BitConverter.ToInt16(new byte[] { payload[9], payload[8] }, 0) * 7.0 * Math.PI / Math.Pow(2, 16);
                            //then z
                            zRateCorrected = BitConverter.ToInt16(new byte[] { payload[11], payload[10] }, 0) * 7.0 * Math.PI / Math.Pow(2, 16);
                            //acceleration * 20 / 2 ^ 16
                            xAccel = BitConverter.ToInt16(new byte[] { payload[1], payload[0] }, 0) * 20.0 / Math.Pow(2, 16);
                            //then y accel
                            yAccel = BitConverter.ToInt16(new byte[] { payload[3], payload[2] }, 0) * 20.0 / Math.Pow(2, 16);
                            //then z accel
                            zAccel = BitConverter.ToInt16(new byte[] { payload[5], payload[4] }, 0) * 20.0 / Math.Pow(2, 16);
                            //xRateTemp, 2 unsigned bytes
                            xRateTemp = BitConverter.ToInt16(new byte[] { payload[13], payload[12], }, 0) * (int)(200 / Math.Pow(2, 16));
                            //yRateTemp, 2 unsigned bytes
                            yRateTemp = BitConverter.ToInt16(new byte[] { payload[15], payload[14], }, 0) * (int)(200 / Math.Pow(2, 16));
                            //zRateTemp, 2 unsigned bytes
                            zRateTemp = BitConverter.ToInt16(new byte[] { payload[17], payload[16], }, 0) * (int)(200 / Math.Pow(2, 16));
                            //BoardTemp, 2 unsigned bytes
                            boardTemp = BitConverter.ToInt16(new byte[] { payload[19], payload[18], }, 0) * (double)(200 / Math.Pow(2, 16));
                            //BoardTemp, 2 unsigned bytes
                            double Counter = BitConverter.ToInt16(new byte[] { payload[21], payload[20], }, 0) * (int)(200 / Math.Pow(2, 16));
                            //bit status
                            bitstatus = BitConverter.ToInt16(new byte[] { payload[23], payload[22], }, 0);
                            if (debug)
                            {
                                Console.WriteLine("Roll: " + rollDAngle);
                                Console.WriteLine("Pitch: " + pitchDAngle);
                                Console.WriteLine("Yaw: " + yawSAngle);
                                Console.WriteLine("X Accel: " + xAccel);
                                Console.WriteLine("Y Accel: " + yAccel);
                                Console.WriteLine("Z Accel: " + zAccel);
                                Console.WriteLine("Longitude: " + longitudeGPS);
                                Console.WriteLine("Latitude:  " + latitudeGPS);
                            }

                            break;
                        case 0x5332: //0x5332, 'S2', Scaled Sensor 2 Packet
                            StringBuilder strinn = new StringBuilder();
                            strinn.Append(BitConverter.ToString(payload));
                            if (debug) Console.WriteLine(strinn);
                            //this is the Scaled Sensor 2 packet
                            //it's 28 bits long
                            //It's also chock full of sweet, black tar velocity data.
                            //The first 12th byte is a 4 byte int, the roll angle in radians, scaled by 7*pi/ 2^32 
                            rollDAngle = BitConverter.ToInt32(new byte[] { payload[15], payload[14], payload[13], payload[12] }, 0) * 7.0 * Math.PI / Math.Pow(2, 32);
                            //Then pitch
                            pitchDAngle = BitConverter.ToInt32(new byte[] { payload[19], payload[18], payload[17], payload[16] }, 0) * 7.0 * Math.PI / Math.Pow(2, 32);
                            //then yaw (true north)
                            yawSAngle = BitConverter.ToInt32(new byte[] { payload[23], payload[22], payload[21], payload[20] }, 0) * 7.0 * Math.PI / Math.Pow(2, 32);
                            //then north velocity scaled by 200/2^16 in meter per second
                            xDVel = BitConverter.ToInt32(new byte[] { payload[3], payload[2], payload[1], payload[0] }, 0) * 200.0 / Math.Pow(2, 32);
                            //then east velocity
                            yDVel = BitConverter.ToInt32(new byte[] { payload[7], payload[6], payload[5], payload[4] }, 0) * 200.0 / Math.Pow(2, 32);
                            //then down velocity
                            zDVel = BitConverter.ToInt32(new byte[] { payload[11], payload[10], payload[9], payload[8] }, 0) * 200.0 / Math.Pow(2, 32);
                            bitstatus = BitConverter.ToInt16(new byte[] { payload[27], payload[26], }, 0);
                            if (debug)
                            {
                                Console.WriteLine("Roll: " + rollDAngle);
                                Console.WriteLine("Pitch: " + pitchDAngle);
                                Console.WriteLine("Yaw: " + yawSAngle);
                                Console.WriteLine("X Accel: " + xAccel);
                                Console.WriteLine("Y Accel: " + yAccel);
                                Console.WriteLine("Z Accel: " + zAccel);
                                Console.WriteLine("Longitude: " + longitudeGPS);
                                Console.WriteLine("Latitude:  " + latitudeGPS);
                            }

                            break;
                        case 0x4E31: //0x4E31, 'N1', Nav Data Packet 1
                            StringBuilder stri = new StringBuilder();
                            stri.Append(BitConverter.ToString(payload));
                            if (debug) Console.WriteLine(stri);
                            //This is it baby! The packet with all that sweet, sweet data.
                            //The payload is 32 bytes in length.
                            //The first 2 bytes is a 2 byte int, the roll angle in radians, scaled by 2*pi/ 2^16 
                            rollDAngle = BitConverter.ToInt16(new byte[] { payload[1], payload[0] }, 0) * 2.0 * Math.PI / Math.Pow(2, 16);
                            //Then pitch
                            pitchDAngle = BitConverter.ToInt16(new byte[] { payload[3], payload[2] }, 0) * 2.0 * Math.PI / Math.Pow(2, 16);
                            //then yaw (true north)
                            yawSAngle = BitConverter.ToInt16(new byte[] { payload[5], payload[4] }, 0) * 2.0 * Math.PI / Math.Pow(2, 16);
                            //then the xRateCorrected in radians per sec, the x angular rate corrected
                            xRateCorrected = BitConverter.ToInt16(new byte[] { payload[7], payload[6] }, 0) * 7.0 * Math.PI / Math.Pow(2, 16);
                            //then yRateCorrected
                            yRateCorrected = BitConverter.ToInt16(new byte[] { payload[9], payload[8] }, 0) * 7.0 * Math.PI / Math.Pow(2, 16);
                            //then z
                            zRateCorrected = BitConverter.ToInt16(new byte[] { payload[11], payload[10] }, 0) * 7.0 * Math.PI / Math.Pow(2, 16);
                            //acceleration * 20 / 2 ^ 16
                            xAccel = BitConverter.ToInt16(new byte[] { payload[13], payload[12] }, 0) * 20.0 / Math.Pow(2, 16);
                            //then east velocity
                            yAccel = BitConverter.ToInt16(new byte[] { payload[15], payload[14] }, 0) * 20.0 / Math.Pow(2, 16);
                            //then down velocity
                            zAccel = BitConverter.ToInt16(new byte[] { payload[17], payload[16] }, 0) * 20.0 / Math.Pow(2, 16);
                            //then north velocity scaled by 512/2^16 in meter per second
                            nVel = BitConverter.ToInt16(new byte[] { payload[19], payload[18] }, 0) * 512.0 / Math.Pow(2, 16);
                            //then east velocity
                            eVel = BitConverter.ToInt16(new byte[] { payload[21], payload[20] }, 0) * 512.0 / Math.Pow(2, 16);
                            //then down velocity
                            dVel = BitConverter.ToInt16(new byte[] { payload[23], payload[22] }, 0) * 512.0 / Math.Pow(2, 16);
                            //GPS logitude in radians scaled by 2*pi/2^32
                            longitudeGPS = BitConverter.ToInt32(new byte[] { payload[25], payload[24], payload[27], payload[26] }, 0) * 2.0 * Math.PI / Math.Pow(2, 32);
                            //GPS latitude
                            latitudeGPS = BitConverter.ToInt32(new byte[] { payload[29], payload[28], payload[31], payload[30] }, 0) * 2.0 * Math.PI / Math.Pow(2, 32);
                            //GPS altitude
                            altitudeGPS = BitConverter.ToInt16(new byte[] { payload[33], payload[32] }, 0) * Math.Pow(2, 14) / Math.Pow(2, 16);
                            //xRateTemp, 2 unsigned bytes
                            double xRateTempr = BitConverter.ToInt16(new byte[] { payload[35], payload[34], }, 0) * (int)(200 / Math.Pow(2, 16));
                            //time ITOW
                            double timeitow = BitConverter.ToInt32(new byte[] { payload[37], payload[36], payload[39], payload[38] }, 0);
                            //bit status
                            bitstatus = BitConverter.ToInt16(new byte[] { payload[41], payload[40], }, 0);
                            if (debug)
                            {
                                Console.WriteLine("Roll: " + rollDAngle);
                                Console.WriteLine("Pitch: " + pitchDAngle);
                                Console.WriteLine("Yaw: " + yawSAngle);
                                Console.WriteLine("X Accel: " + xAccel);
                                Console.WriteLine("Y Accel: " + yAccel);
                                Console.WriteLine("Z Accel: " + zAccel);
                                Console.WriteLine("Longitude: " + longitudeGPS);
                                Console.WriteLine("Latitude:  " + latitudeGPS);
                            }

                            break;
                    }
                }
            }
            //Console.WriteLine(Encoding.UTF8.GetString(incoming));
            if (debug)
            {
                StringBuilder str = new StringBuilder(BitConverter.ToString(incoming));
                //str.Append(BitConverter.ToString(incoming));
                Console.WriteLine(str.ToString());
            }
        }

        public void getPacketRequest(byte type1, byte type2)
        {
            //sends the get packet request for a type of packet
            byte[] request = new byte[8];
            request[0] = preamble[0];
            request[1] = preamble[1];
            request[2] = 0x47;
            request[3] = 0x50;
            request[4] = type1;
            request[5] = type2;
            byte[] crc = calculateCRC(request);
            request[6] = crc[0];
            request[7] = crc[1];
            nav.Send(request);
        }

        public void getID()
        {
            getPacketRequest(0x49, 0x44);
        }

        public void getData()
        {
            getPacketRequest(0x4E, 0x30);
        }


        byte[] calculateCRC(byte[] packet)
        {
            /*******************************************************************************
            * FUNCTION: calcCRC calculates a 2-byte CRC on serial data using
            Page 112 NAV440 User Manual
            7430‐0131‐01 Rev. F
            * CRC-CCITT 16-bit standard maintained by the ITU
            * (International Telecommunications Union).
            * ARGUMENTS: queue_ptr is pointer to queue holding area to be CRCed
            * startIndex is offset into buffer where to begin CRC calculation
            * num is offset into buffer where to stop CRC calculation
            * RETURNS: 2-byte CRC
            ******************************************************************************
            unsigned short calcCRC(QUEUE_TYPE* queue_ptr, unsigned int startIndex, unsigned int num) {
                unsigned int i = 0, j = 0;
                unsigned short crc = 0x1D0F; //non-augmented inital value equivalent to augmented initial value 0xFFFF
            for (i = 0; i < num; i += 1)
                {
                    crc ^= peekByte(queue_ptr, startIndex + i) << 8;
                    for (j = 0; j < 8; j += 1)
                    {
                        if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
                        else crc = crc << 1;
                    }
                }
                return crc;
            }*/

            ushort crc = 0x1D0F;
            for (int i = 2; i < packet.Length - 2; i++)
            {
                crc ^= (ushort)(packet[i] << 8);

                for (int j = 0; j < 8; j++)
                {
                    if ((crc & 0x8000) == 0x1000)
                        crc = (ushort)((crc << 1) ^ 0x1021);
                    else
                        crc = (ushort)(crc << 1);
                }
            }
            byte[] bytes = BitConverter.GetBytes(crc);

            return bytes;
        }

        public IMU440(string port, int baudrate, log loger, bool debug = false)
        {
            lg = loger;
            nav = new Serial(port, baudrate);
            this.debug = debug;
            nav.PackageMode = Serial.PackageModes.UseFunction;
            nav.EscapeToken = new byte[0];
            nav.FindPackageEnd = (bytes =>
            {
                int offset = 0;
                for (int i = 0; i < bytes.Length - 1; i++)
                {

                    if (bytes[i] == 0x55 && bytes[i + 1] == 0x55)
                    {

                        offset = i;
                    }
                }
                if (bytes.Length >= 5 && bytes[offset] == 0x55 && bytes[offset + 1] == 0x55)
                {
                    if (bytes.Length > offset + 4 &&
                        bytes.Length >= (offset + 7 + bytes[offset + 4]))
                    {
                        return offset + 7 + bytes[offset + 4];
                    }
                }
                return -1;
            });
            nav.PackageReceived = navCallback;
            nav.Start();
        }
    }
}