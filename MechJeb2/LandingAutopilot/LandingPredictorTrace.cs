using System;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace MuMech.Landing
{
    internal sealed class TerrainProfileTrace
    {
        public bool Applied;
        public AbsoluteVector SimulatorEndpoint;
        public double SimulatorEndpointASL = double.NaN;
        public int FirstProfileIndex = -1;
        public int ProfileSampleCount;
        public int LocalContactIndex = -1;
        public int ContactIndex = -1;
        public double ContactTerrainASL = double.NaN;
        public int TerrainQueryCount;
        public double TerrainQueryElapsedMs;
        public string CaptureSamples;
    }

    /// <summary>
    /// Writes compact, event-driven V1 predictor diagnostics. It records values
    /// already created by the simulator and terrain resolver; it does not run
    /// another simulation or enumerate the trajectory for logging.
    /// </summary>
    internal static class LandingPredictorTrace
    {
        private static readonly object Sync = new object();
        private static StreamWriter _writer;
        private static int _recordsSinceFlush;

        public static void Write(string decision, ReentrySimulation.Result result, TerrainProfileTrace terrain,
            string phase, string status, double processUT, double simulationSeconds,
            string targetBody, double targetLatitude, double targetLongitude)
        {
            if (result == null)
                return;

            try
            {
                lock (Sync)
                {
                    EnsureWriter();
                    if (_writer == null)
                        return;

                    AbsoluteVector start = result.StartPosition;
                    AbsoluteVector simulatedEnd = terrain == null ? result.EndPosition : terrain.SimulatorEndpoint;
                    double simulatedEndASL = terrain == null ? result.EndASL : terrain.SimulatorEndpointASL;
                    AbsoluteVector trajectoryStart = result.Trajectory != null && result.Trajectory.Count > 0
                        ? result.Trajectory[0]
                        : default(AbsoluteVector);
                    bool hasTrajectoryStart = result.Trajectory != null && result.Trajectory.Count > 0;

                    Vector3d inputVelocity = result.InputInitialOrbit == null
                        ? Vector3d.zero
                        : result.InputInitialOrbit.WorldOrbitalVelocityAtUT(result.InputUT);
                    Vector3d osculatingPosition = OsculatingImpactPosition(result, out double osculatingImpactUT);
                    bool hasOsculatingImpact = !double.IsNaN(osculatingImpactUT) && !double.IsInfinity(osculatingImpactUT);

                    string line = string.Format(CultureInfo.InvariantCulture,
                        "{{\"recordType\":\"predictor_result\",\"decision\":{0},\"processUT\":{1},\"phase\":{2},\"status\":{3}," +
                        "\"body\":{4},\"targetLat\":{5},\"targetLon\":{6},\"outcome\":{7},\"inputUT\":{8},\"resultAgeSeconds\":{9},\"endUT\":{10},\"simulationSeconds\":{11},\"steps\":{12}," +
                        "\"inputDecelerationEndASL\":{13},\"inputMaximumThrustAcceleration\":{14},\"inputMaxOrbits\":{15}," +
                        "\"inputNoSkipToFreefall\":{16},\"inputDescentSpeedPolicy\":{17},\"inputForcedBrakingStartUT\":{18}," +
                        "\"simulatedBrakingStartUT\":{19},\"simulatedBrakingDeltaV\":{20},\"endSurfaceSpeed\":{21}," +
                        "\"inputStart\":[{22},{23},{24},{25}],\"inputVelocity\":[{26},{27},{28}]," +
                        "\"simulatorEndpoint\":[{29},{30},{31},{32}],\"simulatorTargetError\":{33}," +
                        "\"resolvedEndpoint\":[{34},{35},{36},{37}],\"resolvedTargetError\":{38}," +
                        "\"terrainProfileApplied\":{39},\"terrainProfileStartIndex\":{40},\"terrainProfileSampleCount\":{41}," +
                        "\"terrainProfileLocalContactIndex\":{42},\"terrainProfileContactIndex\":{43},\"terrainProfileContactASL\":{44}," +
                        "\"trajectoryStart\":{45},\"osculatingImpact\":{46},\"osculatingImpactUT\":{47}}}",
                        JsonString(decision), Number(processUT), JsonString(phase), JsonString(status), JsonString(targetBody),
                        Number(targetLatitude), Number(targetLongitude), JsonString(result.Outcome.ToString()),
                        Number(result.InputUT), Number(processUT - result.InputUT), Number(result.EndUT), Number(simulationSeconds), result.Steps,
                        Number(result.InputDecelEndAltitudeASL), Number(result.InputMaxThrustAccel), Number(result.InputMaxOrbits),
                        result.InputNoSkipToFreefall ? "true" : "false", JsonString(result.InputDescentSpeedPolicy?.GetType().Name), Number(result.InputForcedBrakingStartUT),
                        Number(result.SimulatedBrakingStartUT), Number(result.DeltaVExpended), Number(result.EndSurfaceSpeed),
                        Number(start.Latitude), Number(start.Longitude), Number(start.Radius), Number(start.UT),
                        Number(inputVelocity.x), Number(inputVelocity.y), Number(inputVelocity.z),
                        Number(simulatedEnd.Latitude), Number(simulatedEnd.Longitude), Number(simulatedEnd.Radius), Number(simulatedEndASL),
                        Number(TargetDistance(result.Body, simulatedEnd, targetLatitude, targetLongitude)),
                        Number(result.EndPosition.Latitude), Number(result.EndPosition.Longitude), Number(result.EndPosition.Radius), Number(result.EndASL),
                        Number(TargetDistance(result.Body, result.EndPosition, targetLatitude, targetLongitude)),
                        terrain != null && terrain.Applied ? "true" : "false", terrain?.FirstProfileIndex ?? -1, terrain?.ProfileSampleCount ?? 0,
                        terrain?.LocalContactIndex ?? -1, terrain?.ContactIndex ?? -1, Number(terrain?.ContactTerrainASL ?? double.NaN),
                        hasTrajectoryStart ? Vector(trajectoryStart) : "null", hasOsculatingImpact ? Vector(osculatingPosition) : "null", Number(osculatingImpactUT));
                    _writer.WriteLine(line);
                    _recordsSinceFlush++;
                    if (_recordsSinceFlush >= 5)
                    {
                        _writer.Flush();
                        _recordsSinceFlush = 0;
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.Log("[MechJebLandingTrace] predictor trace write failed: " + ex.Message);
                Close();
            }
        }

        public static void Close()
        {
            lock (Sync)
            {
                if (_writer == null)
                    return;

                try { _writer.Dispose(); }
                finally
                {
                    _writer = null;
                    _recordsSinceFlush = 0;
                }
            }
        }

        public static void WriteState(string phase, string status, double ut, double warpRate,
            Vector3d position, Vector3d velocity, double commandedThrottle, double thrustAcceleration,
            long predictionVersion, ReentrySimulation.Result prediction, string controllerDetail,
            VesselState vesselState, double attitudeError, string thrustMode,
            double thrustSpeedSetpoint, bool thrustKillHorizontal,
            string recordType = "guidance_state")
        {
            try
            {
                lock (Sync)
                {
                    EnsureWriter();
                    if (_writer == null)
                        return;

                    string line = string.Format(CultureInfo.InvariantCulture,
                        "{{\"recordType\":" + JsonString(recordType) + ",\"ut\":{0},\"phase\":{1},\"status\":{2},\"warpRate\":{3}," +
                        "\"position\":{4},\"velocity\":{5},\"commandedThrottle\":{6},\"thrustAcceleration\":{7}," +
                        "\"predictionVersion\":{8},\"predictionInputUT\":{9},\"predictionEndUT\":{10}," +
                        "\"predictionEndpoint\":{11},\"predictionEndASL\":{12},\"controllerDetail\":{13}," +
                        "\"altitudeASL\":{14},\"altitudeTrue\":{15},\"altitudeBottom\":{16}," +
                        "\"mass\":{17},\"limitedMaxThrustAcceleration\":{18},\"maxThrustAcceleration\":{19}," +
                        "\"localGravity\":{20},\"forward\":{21},\"up\":{22},\"surfaceVelocity\":{23}," +
                        "\"gravityForce\":{24},\"attitudeErrorDegrees\":{25}," +
                        "\"thrustMode\":{26},\"thrustSpeedSetpoint\":{27},\"thrustKillHorizontal\":{28}}}",
                        Number(ut), JsonString(phase), JsonString(status), Number(warpRate), Vector(position), Vector(velocity),
                        Number(commandedThrottle), Number(thrustAcceleration), predictionVersion,
                        Number(prediction?.InputUT ?? double.NaN), Number(prediction?.EndUT ?? double.NaN),
                        prediction == null ? "null" : Vector(prediction.EndPosition), Number(prediction?.EndASL ?? double.NaN),
                        JsonString(controllerDetail), Number(vesselState.AltitudeASL),
                        Number(vesselState.AltitudeTrue), Number(vesselState.AltitudeBottom),
                        Number(vesselState.Mass), Number(vesselState.LimitedMaxThrustAcceleration),
                        Number(vesselState.MaxThrustAcceleration), Number(vesselState.LocalGravity),
                        Vector(vesselState.Forward), Vector(vesselState.Up), Vector(vesselState.SurfaceVelocity),
                        Vector(vesselState.GravityForce), Number(attitudeError), JsonString(thrustMode),
                        Number(thrustSpeedSetpoint), thrustKillHorizontal ? "true" : "false");
                    _writer.WriteLine(line);
                    _recordsSinceFlush++;
                    if (_recordsSinceFlush >= 5)
                    {
                        _writer.Flush();
                        _recordsSinceFlush = 0;
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.Log("[MechJebLandingTrace] state trace write failed: " + ex.Message);
                Close();
            }
        }

        private static void EnsureWriter()
        {
            if (_writer != null)
                return;

            string path = MuUtils.GetCfgPath("LandingGuidanceV1.trace.jsonl");
            string directory = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(directory))
                Directory.CreateDirectory(directory);
            _writer = new StreamWriter(OpenAppendStream(path));
        }

        // A forced KSP exit can leave the last buffered line incomplete.  Before a
        // new trace session appends, remove only that unterminated fragment so the
        // next record starts on its own JSONL line.
        private static FileStream OpenAppendStream(string path)
        {
            var stream = new FileStream(path, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.Read);
            if (stream.Length > 0)
            {
                stream.Seek(-1, SeekOrigin.End);
                if (stream.ReadByte() != '\n')
                {
                    long newline = FindLastNewline(stream);
                    stream.SetLength(newline + 1);
                }
            }
            stream.Seek(0, SeekOrigin.End);
            return stream;
        }

        private static long FindLastNewline(FileStream stream)
        {
            const int bufferSize = 4096;
            byte[] buffer = new byte[bufferSize];
            long end = stream.Length;
            while (end > 0)
            {
                int count = (int)Math.Min(bufferSize, end);
                end -= count;
                stream.Seek(end, SeekOrigin.Begin);
                int read = stream.Read(buffer, 0, count);
                for (int i = read - 1; i >= 0; i--)
                    if (buffer[i] == '\n') return end + i;
            }
            return -1;
        }

        private static double TargetDistance(CelestialBody body, AbsoluteVector endpoint, double targetLatitude, double targetLongitude)
        {
            if (body == null || double.IsNaN(targetLatitude) || double.IsNaN(targetLongitude))
                return double.NaN;

            Vector3d endpointPosition = body.GetWorldSurfacePosition(endpoint.Latitude, endpoint.Longitude, 0);
            Vector3d targetPosition = body.GetWorldSurfacePosition(targetLatitude, targetLongitude, 0);
            return Vector3d.Distance(endpointPosition, targetPosition);
        }
        private static Vector3d OsculatingImpactPosition(ReentrySimulation.Result result, out double impactUT)
        {
            impactUT = double.NaN;
            if (result.InputInitialOrbit == null || result.Body == null)
                return Vector3d.zero;

            try
            {
                double radius = result.Body.Radius + result.InputDecelEndAltitudeASL - 100.0;
                Orbit orbit = result.InputInitialOrbit;
                impactUT = orbit.PeR < radius
                    ? orbit.NextTimeOfRadius(result.InputUT, radius)
                    : orbit.NextPeriapsisTime(result.InputUT);
                if (double.IsNaN(impactUT) || double.IsInfinity(impactUT))
                    return Vector3d.zero;
                return orbit.WorldBCIPositionAtUT(impactUT);
            }
            catch
            {
                impactUT = double.NaN;
                return Vector3d.zero;
            }
        }

        private static string Vector(AbsoluteVector value) => string.Format(CultureInfo.InvariantCulture,
            "[{0},{1},{2},{3}]", Number(value.Latitude), Number(value.Longitude), Number(value.Radius), Number(value.UT));

        private static string Vector(Vector3d value) => string.Format(CultureInfo.InvariantCulture,
            "[{0},{1},{2}]", Number(value.x), Number(value.y), Number(value.z));

        private static string Number(double value) => double.IsNaN(value) || double.IsInfinity(value)
            ? "null"
            : value.ToString("R", CultureInfo.InvariantCulture);

        private static string JsonString(string value) => value == null
            ? "null"
            : "\"" + value.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
    }
}
