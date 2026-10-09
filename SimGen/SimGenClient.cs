using System;
using System.Globalization;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace D2S2.Testing
{
    public class SimGenClient
    {
        private const int Port = 15650;

        private TcpClient client;
        private NetworkStream stream;


        public void Connect(string ipAddress)
        {
            client = new TcpClient();

            client.Connect(
                ipAddress,
                Port
            );

            stream = client.GetStream();

            stream.ReadTimeout = 5000;
            stream.WriteTimeout = 5000;
        }


        public void Close()
        {
            if (stream != null)
            {
                stream.Close();
                stream = null;
            }

            if (client != null)
            {
                client.Close();
                client = null;
            }
        }


        public void Send(string command)
        {
            if (stream == null)
            {
                throw new Exception(
                    "SimGEN connection is not open.");
            }

            byte[] data =
                Encoding.ASCII.GetBytes(command);

            stream.Write(
                data,
                0,
                data.Length
            );

            stream.Flush();
        }


        public string Receive()
        {
            if (stream == null)
            {
                throw new Exception(
                    "SimGEN connection is not open.");
            }

            byte[] buffer =
                new byte[4096];

            int bytesRead =
                stream.Read(
                    buffer,
                    0,
                    buffer.Length
                );

            return Encoding.ASCII.GetString(
                buffer,
                0,
                bytesRead
            );
        }


        private string GetTag(
            string text,
            string tag)
        {
            string start =
                "<" + tag + ">";

            string end =
                "</" + tag + ">";

            int startIndex =
                text.IndexOf(start);

            if (startIndex < 0)
            {
                return "";
            }

            startIndex +=
                start.Length;

            int endIndex =
                text.IndexOf(
                    end,
                    startIndex
                );

            if (endIndex < 0)
            {
                return "";
            }

            return text.Substring(
                startIndex,
                endIndex - startIndex
            );
        }


        public int GetStatus()
        {
            Send("NULL\r");

            string response =
                Receive();

            string status =
                GetTag(
                    response,
                    "status"
                );

            if (status.Length == 0)
            {
                throw new Exception(
                    "Could not find SimGEN status in response: " +
                    response);
            }

            return int.Parse(
                status,
                CultureInfo.InvariantCulture
            );
        }


        public void SetTriggerMode()
        {
            Send("TR,0\r");
        }


        public void SetStartTime(
            DateTime time)
        {
            DateTime start =
                time;

            int remainder =
                start.Second % 6;

            if (remainder != 0)
            {
                start =
                    start.AddSeconds(
                        6 - remainder
                    );
            }

            string month =
                start.ToString(
                    "MMM",
                    CultureInfo.InvariantCulture
                ).ToUpperInvariant();

            string timestamp =
                string.Format(
                    CultureInfo.InvariantCulture,

                    "{0:00}-{1}-{2:0000} " +
                    "{3:00}:{4:00}:{5:00}",

                    start.Day,
                    month,
                    start.Year,

                    start.Hour,
                    start.Minute,
                    start.Second
                );

            string command =
                "START_TIME," +
                timestamp +
                ",1 00:00:00\r";

            Send(command);
        }


        public void Arm()
        {
            Send("AR_NOWAIT\r");
        }


        public void Run()
        {
            Send("RU\r");
        }


        public void End()
        {
            Send("-,EN\r");
        }


        public void WaitForStatus(
            int wantedStatus)
        {
            while (true)
            {
                int status =
                    GetStatus();

                if (status == wantedStatus)
                {
                    return;
                }

                Thread.Sleep(250);
            }
        }


        // ----------------------------------------------------
        // MOTQ
        //
        // Position:
        //   WGS84 ECEF, metres
        //
        // Velocity:
        //   WGS84 ECEF, metres/second
        //
        // Quaternion:
        //   q0 = scalar
        //   q1 = X
        //   q2 = Y
        //   q3 = Z
        //
        // The quaternion describes vehicle attitude relative
        // to the local geodetic NED frame.
        //
        // Acceleration and jerk are currently set to zero.
        // Angular-rate fields are omitted from the end.
        // ----------------------------------------------------
        public void SendMotionQuaternion(
            double elapsedSeconds,

            double x,
            double y,
            double z,

            double vx,
            double vy,
            double vz,

            double q0,
            double q1,
            double q2,
            double q3)
        {
            long totalMilliseconds =
                (long)Math.Round(
                    elapsedSeconds * 1000.0);

            int milliseconds =
                (int)(
                    totalMilliseconds % 1000);

            long totalSeconds =
                totalMilliseconds / 1000;

            int seconds =
                (int)(
                    totalSeconds % 60);

            long totalMinutes =
                totalSeconds / 60;

            int minutes =
                (int)(
                    totalMinutes % 60);

            long hours =
                totalMinutes / 60;


            string motionTime =
                string.Format(
                    CultureInfo.InvariantCulture,

                    "{0:00}:{1:00}:{2:00}.{3:000}",

                    hours,
                    minutes,
                    seconds,
                    milliseconds
                );


            // MOTQ requires acceleration and jerk fields
            // before the quaternion. For now we explicitly
            // send zero for all six.
            string command =
                string.Format(
                    CultureInfo.InvariantCulture,

                    "{0},MOTQ,v1_m1," +

                    // ECEF position
                    "{1:R},{2:R},{3:R}," +

                    // ECEF velocity
                    "{4:R},{5:R},{6:R}," +

                    // ECEF acceleration
                    "0,0,0," +

                    // ECEF jerk
                    "0,0,0," +

                    // Quaternion
                    "{7:R},{8:R},{9:R},{10:R}\r",

                    motionTime,

                    x,
                    y,
                    z,

                    vx,
                    vy,
                    vz,

                    q0,
                    q1,
                    q2,
                    q3
                );


            Send(command);
        }
    }
}