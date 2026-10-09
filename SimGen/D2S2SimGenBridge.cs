using D2S2.Model;
using D2S2.Model.Satellite;
using D2S2.Shared.MathPhysics;
using D2S2.Shared.Satellite;
using D2S2.Simulation;

using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.Threading;

namespace D2S2.Testing
{
    [DisplayName("D2S2 -> SimGEN")]
    public class D2S2SimGenBridge
        : SimulationScript
    {
        // Replace with your actual SimGEN address.
        private const string SimGenIp =
            "192.168.150.177";


        private SatelliteModelBase sat;

        private SimGenClient simgen;

        private Stopwatch realTimeClock;

        private bool firstIteration;

        private DateTime simulationStart;


        // ====================================================
        // Run
        // ====================================================
        protected override void Run()
        {
            ReportProgress(
                "Starting D2S2 -> SimGEN bridge...");


            SatelliteSimulation simRoot =
                (SatelliteSimulation)
                SimulationManager.Model;


            if (simRoot.Satellites.Count == 0)
            {
                throw new Exception(
                    "No satellites found.");
            }


            sat =
                simRoot.Satellites[0];


            ReportProgress(
                "Satellite: " +
                sat.Name);


            simulationStart =
                SimulationControlService
                    .SimulationStart;


            simgen =
                new SimGenClient();


            ReportProgress(
                "Connecting to SimGEN at " +
                SimGenIp +
                "...");


            simgen.Connect(
                SimGenIp);


            ReportProgress(
                "Connected to SimGEN.");


            firstIteration = true;

            realTimeClock =
                new Stopwatch();


            try
            {
                RunSimulation(null);
            }
            finally
            {
                if (simgen != null)
                {
                    simgen.Close();
                }
            }


            ReportProgress(
                "D2S2 -> SimGEN bridge finished.");
        }


        // ====================================================
        // Build NED -> ECEF DCM
        //
        // The columns of this matrix are the North, East and
        // Down unit vectors expressed in ECEF coordinates.
        // ====================================================
        private Dcm BuildNedToEcef(
            double latitude,
            double longitude)
        {
            double sinLat =
                Math.Sin(latitude);

            double cosLat =
                Math.Cos(latitude);

            double sinLon =
                Math.Sin(longitude);

            double cosLon =
                Math.Cos(longitude);


            Dcm result =
                Dcm.Identity;


            // North axis in ECEF
            result.M11 =
                -sinLat * cosLon;

            result.M21 =
                -sinLat * sinLon;

            result.M31 =
                cosLat;


            // East axis in ECEF
            result.M12 =
                -sinLon;

            result.M22 =
                cosLon;

            result.M32 =
                0.0;


            // Down axis in ECEF
            result.M13 =
                -cosLat * cosLon;

            result.M23 =
                -cosLat * sinLon;

            result.M33 =
                -sinLat;


            return result;
        }


        // ====================================================
        // Wrap an angle into [-PI, PI].
        // ====================================================
        private double WrapAngle(
            double angle)
        {
            while (angle > Math.PI)
            {
                angle -=
                    2.0 * Math.PI;
            }

            while (angle < -Math.PI)
            {
                angle +=
                    2.0 * Math.PI;
            }

            return angle;
        }


        // ====================================================
        // Obtain geodetic latitude/longitude from D2S2 and
        // ensure we work internally in radians.
        //
        // D2S2's ECEF longitude can be independently checked
        // against atan2(Y,X), allowing us to detect whether
        // the returned Lat/Lon values are radians or degrees.
        // ====================================================
        private void GetGeodeticRadians(
            Vector3d positionEcef,
            out double latitude,
            out double longitude)
        {
            double altitude;


            EarthCoordinateConversion
                .EcefToLatLonAlt(
                    positionEcef,

                    out latitude,
                    out longitude,
                    out altitude
                );


            // Geocentric/ECEF longitude is simply atan2(Y,X).
            double expectedLongitude =
                Math.Atan2(
                    positionEcef.Y,
                    positionEcef.X
                );


            // Compare assuming D2S2 returned radians.
            double radiansError =
                Math.Abs(
                    WrapAngle(
                        longitude -
                        expectedLongitude
                    )
                );


            // Compare assuming D2S2 returned degrees.
            double longitudeAsRadians =
                longitude *
                Math.PI /
                180.0;


            double degreesError =
                Math.Abs(
                    WrapAngle(
                        longitudeAsRadians -
                        expectedLongitude
                    )
                );


            // Whichever interpretation agrees better with
            // the ECEF coordinates determines the units.
            if (degreesError <
                radiansError)
            {
                latitude =
                    latitude *
                    Math.PI /
                    180.0;

                longitude =
                    longitudeAsRadians;
            }
        }


        // ====================================================
        // Convert D2S2's satellite attitude into the attitude
        // convention required by SimGEN.
        //
        // D2S2 gives:
        //
        //   ECI -> Body
        //
        // We want:
        //
        //   Body -> local NED
        //
        // Therefore:
        //
        // Body -> ECI
        //      -> ECEF
        //      -> NED
        //
        // C_N_B =
        //     C_N_E *
        //     C_E_I *
        //     C_I_B
        //
        // ====================================================
        private Quaternion64 GetSimGenAttitude(
            DateTime simTime,
            Vector3d positionEcef)
        {
            // ------------------------------------------------
            // D2S2:
            // ECI -> Body
            // ------------------------------------------------
            Dcm inertialToBody =
                sat.KinematicState
                    .InertialToBodyTransform;


            // ------------------------------------------------
            // Reverse it:
            // Body -> ECI
            // ------------------------------------------------
            Dcm bodyToInertial =
                Dcm.Transpose(
                    inertialToBody);


            // ------------------------------------------------
            // D2S2 native:
            // ECI -> ECEF
            // ------------------------------------------------
            Dcm inertialToEcef =
                EarthCoordinateConversion
                    .GetEciToEcefTransform(
                        simTime);


            // ------------------------------------------------
            // Determine spacecraft geodetic location.
            // ------------------------------------------------
            double latitude;
            double longitude;


            GetGeodeticRadians(
                positionEcef,

                out latitude,
                out longitude
            );


            // ------------------------------------------------
            // Local NED -> ECEF
            // ------------------------------------------------
            Dcm nedToEcef =
                BuildNedToEcef(
                    latitude,
                    longitude
                );


            // ------------------------------------------------
            // Reverse:
            // ECEF -> local NED
            // ------------------------------------------------
            Dcm ecefToNed =
                Dcm.Transpose(
                    nedToEcef);


            // ------------------------------------------------
            // Body -> NED
            //
            // Body -> ECI -> ECEF -> NED
            // ------------------------------------------------
            Dcm bodyToNed =
                ecefToNed *
                inertialToEcef *
                bodyToInertial;


            // SimGEN wants the inverse attitude:
            // local NED -> Body.
            Dcm nedToBody =
                Dcm.Transpose(
                    bodyToNed);

            // ------------------------------------------------
            // D2S2 can directly convert the final DCM into
            // a Quaternion64.
            // ------------------------------------------------
            Quaternion64 attitude =
                nedToBody.ToQuaternion();


            attitude.Normalize();


            return attitude;
        }


        // ====================================================
        // One D2S2 simulation iteration
        // ====================================================
        public override void SimulationIteration(
            DateTime simTime,
            object userContext)
        {
            if (sat == null ||
                simgen == null)
            {
                return;
            }


            double elapsed =
                (simTime - simulationStart)
                .TotalSeconds;


            // =================================================
            // D2S2 truth state in J2000 ECI
            // =================================================
            Vector3d positionEci =
                sat.KinematicState.Position;


            Vector3d velocityEci =
                sat.KinematicState.Velocity;


            // =================================================
            // Convert position + velocity using D2S2's native
            // ECI -> ECEF implementation.
            // =================================================
            Vector3d positionEcef =
                default(Vector3d);


            Vector3d velocityEcef =
                default(Vector3d);


            EarthCoordinateConversion
                .EciToEcef(
                    simTime,

                    positionEci,
                    velocityEci,

                    out positionEcef,
                    out velocityEcef
                );


            // =================================================
            // Calculate SimGEN-compatible attitude.
            // =================================================
            Quaternion64 attitude =
                GetSimGenAttitude(
                    simTime,
                    positionEcef
                );


            // SimGEN:
            //
            // q0 = scalar
            // q1 = X
            // q2 = Y
            // q3 = Z
            //
            // D2S2 Quaternion64 exposes vector XYZ + scalar W.
            double q0 =
                attitude.W;

            double q1 =
                attitude.X;

            double q2 =
                attitude.Y;

            double q3 =
                attitude.Z;


            // =================================================
            // First sample configures and starts SimGEN.
            // =================================================
            if (firstIteration)
            {
                ReportProgress(
                    "Configuring SimGEN...");


                double quaternionNorm =
                    Math.Sqrt(
                        q0 * q0 +
                        q1 * q1 +
                        q2 * q2 +
                        q3 * q3
                    );


                ReportProgress(
                    string.Format(
                        CultureInfo.InvariantCulture,

                        "Initial SimGEN attitude: " +
                        "q0={0:F6}, " +
                        "q1={1:F6}, " +
                        "q2={2:F6}, " +
                        "q3={3:F6}, " +
                        "|q|={4:F6}",

                        q0,
                        q1,
                        q2,
                        q3,
                        quaternionNorm
                    )
                );


                simgen.SetTriggerMode();


                simgen.SetStartTime(
                    simTime);


                // ---------------------------------------------
                // Initial MOTQ
                // ---------------------------------------------
                simgen.SendMotionQuaternion(
                    elapsed,

                    positionEcef.X,
                    positionEcef.Y,
                    positionEcef.Z,

                    velocityEcef.X,
                    velocityEcef.Y,
                    velocityEcef.Z,

                    q0,
                    q1,
                    q2,
                    q3
                );


                ReportProgress(
                    "Initial MOTQ sent.");


                simgen.Arm();

                Quaternion64 orbitToBody =
                    sat.KinematicState
                        .OrbitToBodyTransform;

                ReportProgress(
                    string.Format(
                        CultureInfo.InvariantCulture,

                        "D2S2 Orbit->Body: " +
                        "X={0:F6}, Y={1:F6}, Z={2:F6}, W={3:F6}",

                        orbitToBody.X,
                        orbitToBody.Y,
                        orbitToBody.Z,
                        orbitToBody.W
                    )
                );


                ReportProgress(
                    "Waiting for SimGEN Armed...");


                // Status 4 = Armed
                simgen.WaitForStatus(4);


                ReportProgress(
                    "SimGEN Armed.");


                simgen.Run();


                ReportProgress(
                    "Waiting for SimGEN Running...");


                // Status 5 = Running
                simgen.WaitForStatus(5);


                ReportProgress(
                    "SimGEN Running.");


                realTimeClock.Start();


                firstIteration = false;


                return;
            }


            // =================================================
            // Pace D2S2 against real time.
            // =================================================
            double wallElapsed =
                realTimeClock
                    .Elapsed
                    .TotalSeconds;


            double waitSeconds =
                elapsed -
                wallElapsed;


            if (waitSeconds > 0.0)
            {
                int waitMilliseconds =
                    (int)Math.Round(
                        waitSeconds *
                        1000.0);


                if (waitMilliseconds > 0)
                {
                    Thread.Sleep(
                        waitMilliseconds);
                }
            }


            // =================================================
            // Send current ECEF state + attitude to SimGEN.
            // =================================================
            simgen.SendMotionQuaternion(
                elapsed,

                positionEcef.X,
                positionEcef.Y,
                positionEcef.Z,

                velocityEcef.X,
                velocityEcef.Y,
                velocityEcef.Z,

                q0,
                q1,
                q2,
                q3
            );
        }


        // ====================================================
        // Don't stop early.
        // ====================================================
        public override bool HaltSimulation(
            DateTime simTime,
            object userContext)
        {
            return false;
        }
    }
}