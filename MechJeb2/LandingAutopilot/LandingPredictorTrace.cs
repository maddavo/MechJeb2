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
                        "\"inputDecelerationEndASL\":{13},\"inputMaximumThrustAcceleration\":{14}," +
                        "\"inputStart\":[{15},{16},{17},{18}],\"inputVelocity\":[{19},{20},{21}]," +
                        "\"simulatorEndpoint\":[{22},{23},{24},{25}],\"resolvedEndpoint\":[{26},{27},{28},{29}]," +
                        "\"terrainProfileApplied\":{30},\"terrainProfileStartIndex\":{31},\"terrainProfileSampleCount\":{32}," +
                        "\"terrainProfileLocalContactIndex\":{33},\"terrainProfileContactIndex\":{34},\"terrainProfileContactASL\":{35}," +
                        "\"trajectoryStart\":{36},\"osculatingImpact\":{37},\"osculatingImpactUT\":{38}}}",
                        JsonString(decision), Number(processUT), JsonString(phase), JsonString(status), JsonString(targetBody),
                        Number(targetLatitude), Number(targetLongitude), JsonString(result.Outcome.ToString()),
                        Number(result.InputUT), Number(processUT - result.InputUT), Number(result.EndUT), Number(simulationSeconds), result.Steps,
                        Number(result.InputDecelEndAltitudeASL), Number(result.InputMaxThrustAccel),
                        Number(start.Latitude), Number(start.Longitude), Number(start.Radius), Number(start.UT),
                        Number(inputVelocity.x), Number(inputVelocity.y), Number(inputVelocity.z),
                        Number(simulatedEnd.Latitude), Number(simulatedEnd.Longitude), Number(simulatedEnd.Radius), Number(simulatedEndASL),
                        Number(result.EndPosition.Latitude), Number(result.EndPosition.Longitude), Number(result.EndPosition.Radius), Number(result.EndASL),
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
            long predictionVersion, ReentrySimulation.Result prediction)
        {
            try
            {
                lock (Sync)
                {
                    EnsureWriter();
                    if (_writer == null)
                        return;

                    string line = string.Format(CultureInfo.InvariantCulture,
                        "{{\"recordType\":\"guidance_state\",\"ut\":{0},\"phase\":{1},\"status\":{2},\"warpRate\":{3}," +
                        "\"position\":{4},\"velocity\":{5},\"commandedThrottle\":{6},\"thrustAcceleration\":{7}," +
                        "\"predictionVersion\":{8},\"predictionInputUT\":{9},\"predictionEndUT\":{10}," +
                        "\"predictionEndpoint\":{11},\"predictionEndASL\":{12}}}",
                        Number(ut), JsonString(phase), JsonString(status), Number(warpRate), Vector(position), Vector(velocity),
                        Number(commandedThrottle), Number(thrustAcceleration), predictionVersion,
                        Number(prediction?.InputUT ?? double.NaN), Number(prediction?.EndUT ?? double.NaN),
                        prediction == null ? "null" : Vector(prediction.EndPosition), Number(prediction?.EndASL ?? double.NaN));
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
            _writer = new StreamWriter(new FileStream(path, FileMode.Append, FileAccess.Write, FileShare.Read));
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
