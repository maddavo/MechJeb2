using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;

namespace MuMech.Landing
{
    // Passive, versioned input/output records for offline airless predictor replay.
    // None of these values are read by the predictor or V1 controller.
    internal static class LandingPredictorCapture
    {
        internal const int SchemaVersion = 1;
        private static readonly string SessionId = Guid.NewGuid().ToString("N");
        private static readonly object Sync = new object();
        private static StreamWriter _writer;
        private static int _recordsSinceFlush;
        private static long _nextId;

        internal sealed class Snapshot
        {
            internal readonly string Fields;

            private Snapshot(string fields) => Fields = fields;

            internal static Snapshot Create(Orbit orbit, double inputUT, CelestialBody body,
                VesselState vesselState, MechJebCore core,
                double targetLatitude, double targetLongitude, string phase, long generation,
                AirlessTargetAwareSnapshot? activeSnapshot = null,
                double? activeTargetTerrainQueryMs = null)
            {
                // All KSP/Unity queries happen on the flight thread, once per predictor submission batch.
                double epoch = activeSnapshot?.EpochUT ?? Planetarium.GetUniversalTime();
                Vector3d position = activeSnapshot?.Position ?? orbit.WorldBCIPositionAtUT(inputUT);
                Vector3d velocity = activeSnapshot?.Velocity ?? orbit.WorldOrbitalVelocityAtUT(inputUT);
                var terrainTimer = new Stopwatch();
                double targetTerrainASL;
                double terrainQueryMs;
                if (activeSnapshot.HasValue)
                {
                    targetTerrainASL = activeSnapshot.Value.TargetTerrainASL;
                    terrainQueryMs = activeTargetTerrainQueryMs ?? 0;
                }
                else
                {
                    terrainTimer.Start();
                    targetTerrainASL = body.TerrainAltitude(targetLatitude, targetLongitude);
                    terrainTimer.Stop();
                    terrainQueryMs = terrainTimer.Elapsed.TotalMilliseconds;
                }
                var fields = new StringBuilder(950);
                fields.Append(",\"generation\":").Append(generation);
                fields.Append(",\"inputUT\":").Append(Number(inputUT));
                fields.Append(",\"captureEpochUT\":").Append(Number(epoch));
                fields.Append(",\"phase\":").Append(Quote(phase));
                fields.Append(",\"body\":").Append(Quote(body.bodyName));
                fields.Append(",\"bodyRadius\":").Append(Number(activeSnapshot?.BodyRadius ?? body.Radius));
                fields.Append(",\"bodyMu\":").Append(Number(activeSnapshot?.BodyMu ?? body.gravParameter));
                fields.Append(",\"bodyGeeASL\":").Append(Number(activeSnapshot?.BodyGeeASL ?? body.GeeASL));
                fields.Append(",\"rotationPeriod\":").Append(Number(activeSnapshot?.RotationPeriod ?? body.rotationPeriod));
                fields.Append(",\"angularVelocity\":").Append(Vector(activeSnapshot?.AngularVelocity ?? body.angularVelocity));
                fields.Append(",\"bodyAxis0\":").Append(Vector(activeSnapshot?.Axis0 ?? body.GetSurfaceNVector(0, 0)));
                fields.Append(",\"bodyAxis90\":").Append(Vector(activeSnapshot?.Axis90 ?? body.GetSurfaceNVector(0, 90)));
                fields.Append(",\"bodyAxisNorth\":").Append(Vector(activeSnapshot?.AxisNorth ?? body.GetSurfaceNVector(90, 0)));
                fields.Append(",\"positionBCI\":").Append(Vector(position));
                fields.Append(",\"velocityBCI\":").Append(Vector(velocity));
                fields.Append(",\"mass\":").Append(Number(vesselState.Mass));
                fields.Append(",\"thrustAvailable\":").Append(Number(vesselState.ThrustAvailable));
                fields.Append(",\"thrustMinimum\":").Append(Number(vesselState.ThrustMinimum));
                fields.Append(",\"throttleFixedLimit\":").Append(Number(vesselState.ThrottleFixedLimit));
                fields.Append(",\"limitedMaxThrustAcceleration\":").Append(Number(vesselState.LimitedMaxThrustAcceleration));
                fields.Append(",\"maxEngineResponseTime\":").Append(Number(vesselState.MaxEngineResponseTime));
                fields.Append(",\"surfaceVelocity\":").Append(Vector(vesselState.SurfaceVelocity));
                fields.Append(",\"orbitalVelocity\":").Append(Vector(vesselState.OrbitalVelocity));
                fields.Append(",\"forward\":").Append(Vector(vesselState.Forward));
                fields.Append(",\"up\":").Append(Vector(vesselState.Up));
                fields.Append(",\"gravityForce\":").Append(Vector(vesselState.GravityForce));
                fields.Append(",\"currentThrustAcceleration\":").Append(Number(vesselState.CurrentThrustAcceleration));
                fields.Append(",\"maxThrustAccelerationAtSubmission\":").Append(Number(vesselState.MaxThrustAcceleration));
                fields.Append(",\"localGravity\":").Append(Number(vesselState.LocalGravity));
                fields.Append(",\"controllerDeltaT\":").Append(Number(vesselState.DeltaT));
                fields.Append(",\"altitudeASL\":").Append(Number(vesselState.AltitudeASL));
                fields.Append(",\"speedSurface\":").Append(Number(vesselState.SpeedSurface));
                fields.Append(",\"speedSurfaceHorizontal\":").Append(Number(vesselState.SpeedSurfaceHorizontal));
                fields.Append(",\"speedVertical\":").Append(Number(vesselState.SpeedVertical));
                fields.Append(",\"attitudeErrorDegrees\":").Append(Number(core.Attitude.attitudeAngleFromTarget()));
                fields.Append(",\"vesselAngularSpeed\":").Append(Number(core.vessel.angularVelocity.magnitude));
                fields.Append(",\"commandedThrottle\":").Append(Number(core.Thrust.TargetThrottle));
                fields.Append(",\"autoWarpEnabled\":").Append(core.Node.Autowarp ? "true" : "false");
                fields.Append(",\"landingTouchdownSpeed\":").Append(Number(core.Landing.TouchdownSpeed));
                fields.Append(",\"rcsAdjustmentEnabled\":").Append(core.Landing.RCSAdjustment ? "true" : "false");
                fields.Append(",\"activePredictionVersion\":").Append(core.Landing.PredictionVersion);
                fields.Append(",\"activePredictionInputUT\":").Append(Number(core.Landing.Prediction?.InputUT ?? double.NaN));
                fields.Append(",\"targetLatitude\":").Append(Number(targetLatitude));
                fields.Append(",\"targetLongitude\":").Append(Number(targetLongitude));
                fields.Append(",\"targetTerrainASL\":").Append(Number(targetTerrainASL));
                fields.Append(",\"targetTerrainQueryElapsedMs\":").Append(Number(terrainQueryMs));
                if (activeSnapshot.HasValue)
                {
                    fields.Append(",\"minimumTerrainASL\":").Append(Number(activeSnapshot.Value.MinimumTerrainASL));
                    fields.Append(",\"maximumTerrainASL\":").Append(Number(activeSnapshot.Value.MaximumTerrainASL));
                    if (activeSnapshot.Value.HasV1ControlModel)
                    {
                        fields.Append(",\"controllerInitialMass\":").Append(Number(activeSnapshot.Value.InitialMass));
                        fields.Append(",\"controllerMaximumThrust\":").Append(Number(activeSnapshot.Value.MaximumThrust));
                        fields.Append(",\"controllerMinimumThrust\":").Append(Number(activeSnapshot.Value.MinimumThrust));
                        fields.Append(",\"controllerMaximumMassFlow\":").Append(Number(activeSnapshot.Value.MaximumMassFlow));
                        fields.Append(",\"controllerMinimumMassFlow\":").Append(Number(activeSnapshot.Value.MinimumMassFlow));
                        fields.Append(",\"controllerPolicyTerrainRadius\":").Append(Number(activeSnapshot.Value.PolicyTerrainRadius));
                        fields.Append(",\"controllerPolicyGravity\":").Append(Number(activeSnapshot.Value.PolicyGravity));
                        fields.Append(",\"controllerPolicyThrust\":").Append(Number(activeSnapshot.Value.PolicyThrust));
                        fields.Append(",\"controllerMinimumCommandThrottle\":").Append(Number(activeSnapshot.Value.MinimumCommandThrottle));
                        fields.Append(",\"controllerMaximumCommandThrottle\":").Append(Number(activeSnapshot.Value.MaximumCommandThrottle));
                        fields.Append(",\"controllerThrottleSmoothingSeconds\":").Append(Number(activeSnapshot.Value.ThrottleSmoothingSeconds));
                        fields.Append(",\"controllerInitialAppliedThrottle\":").Append(Number(activeSnapshot.Value.InitialAppliedThrottle));
                        if (core.Landing.DescentSpeedPolicy is SafeDescentSpeedPolicy livePolicy)
                        {
                            fields.Append(",\"livePolicyTerrainRadius\":").Append(Number(livePolicy.TerrainRadius));
                            fields.Append(",\"livePolicyGravity\":").Append(Number(livePolicy.Gravity));
                            fields.Append(",\"livePolicyThrust\":").Append(Number(livePolicy.Thrust));
                        }
                    }
                }
                return new Snapshot(fields.ToString());
            }
        }

        internal static long Submit(Snapshot snapshot, string kind, IDescentSpeedPolicy policy,
            double decelEndASL, double probableLandingSiteASL, double maxThrustAcceleration,
            double parachuteMultiplier, double dt, double minDt, double maxOrbits, bool noSkipToFreefall,
            double forcedBrakingStartUT, string modelProvenance = null)
        {
            if (snapshot == null) return 0;
            try
            {
            long id = Interlocked.Increment(ref _nextId);
            var line = new StringBuilder(1200);
            line.Append("{\"schemaVersion\":").Append(SchemaVersion).Append(",\"captureSession\":").Append(Quote(SessionId))
                .Append(",\"recordType\":\"submission\",\"submissionId\":").Append(id)
                .Append(",\"wallTimestamp\":").Append(Stopwatch.GetTimestamp())
                .Append(",\"wallTimestampFrequency\":").Append(Stopwatch.Frequency)
                .Append(",\"kind\":").Append(Quote(kind)).Append(snapshot.Fields)
                .Append(",\"modelProvenance\":").Append(Quote(modelProvenance))
                .Append(",\"policy\":").Append(Quote(policy?.GetType().Name))
                .Append(",\"decelEndASL\":").Append(Number(decelEndASL))
                .Append(",\"probableLandingSiteASL\":").Append(Number(probableLandingSiteASL))
                .Append(",\"maxThrustAcceleration\":").Append(Number(maxThrustAcceleration))
                .Append(",\"parachuteMultiplier\":").Append(Number(parachuteMultiplier))
                .Append(",\"dt\":").Append(Number(dt))
                .Append(",\"minDt\":").Append(Number(minDt))
                .Append(",\"maxOrbits\":").Append(Number(maxOrbits))
                .Append(",\"noSkipToFreefall\":").Append(noSkipToFreefall ? "true" : "false")
                .Append(",\"forcedBrakingStartUT\":").Append(Number(forcedBrakingStartUT)).Append('}');
            Write(line.ToString());
            return id;
            }
            catch (Exception) { return 0; }
        }

        // Worker completion uses only copied scalar/result fields. No terrain or Unity API is called here.
        internal static void WorkerResult(long id, ReentrySimulation.Result result, bool staleGeneration,
            double simulationElapsedMs)
        {
            if (id == 0 || result == null) return;
            try
            {
            bool hasEndpoint = result.Outcome == ReentrySimulation.Outcome.LANDED ||
                               result.Outcome == ReentrySimulation.Outcome.AEROBRAKED ||
                               result.Outcome == ReentrySimulation.Outcome.TIMED_OUT;
            bool complete = hasEndpoint && result.Body != null && result.Trajectory != null &&
                            result.Trajectory.Count > 0 && Finite(result.EndUT) &&
                            Finite(result.EndPosition.Latitude) && Finite(result.EndPosition.Longitude) &&
                            Finite(result.EndPosition.Radius) && Finite(result.EndSurfaceSpeed);
            var line = new StringBuilder(550);
            line.Append("{\"schemaVersion\":").Append(SchemaVersion).Append(",\"captureSession\":").Append(Quote(SessionId))
                .Append(",\"recordType\":\"worker_result\",\"submissionId\":").Append(id)
                .Append(",\"wallTimestamp\":").Append(Stopwatch.GetTimestamp())
                .Append(",\"simulationElapsedMs\":").Append(Number(simulationElapsedMs))
                .Append(",\"outcome\":").Append(Quote(result.Outcome.ToString()))
                .Append(",\"staleGeneration\":").Append(staleGeneration ? "true" : "false")
                .Append(",\"complete\":").Append(complete ? "true" : "false")
                .Append(",\"exceptionType\":").Append(Quote(result.Exception?.GetType().Name))
                .Append(",\"steps\":").Append(result.Steps)
                .Append(",\"inputUT\":").Append(Number(result.InputUT))
                .Append(",\"endUT\":").Append(hasEndpoint ? Number(result.EndUT) : "null")
                .Append(",\"virtualBrakeUT\":").Append(hasEndpoint ? Number(result.SimulatedBrakingStartUT) : "null")
                .Append(",\"controllerBrakeReferenceUT\":").Append(
                    result.HasControllerBrakeReferenceUT ? Number(result.ControllerBrakeReferenceUT) : "null")
                .Append(",\"controllerBrakeReferencePosition\":").Append(
                    result.HasControllerBrakeReferenceUT ? Absolute(result.ControllerBrakeReferencePosition) : "null")
                .Append(",\"forcedBrakeUT\":").Append(Number(result.InputForcedBrakingStartUT))
                .Append(",\"start\":").Append(hasEndpoint ? Absolute(result.StartPosition) : "null")
                .Append(",\"simulatorEnd\":").Append(hasEndpoint ? Absolute(result.EndPosition) : "null")
                .Append(",\"simulatorEndVelocity\":").Append(hasEndpoint ? Absolute(result.EndVelocity) : "null")
                .Append(",\"endSurfaceSpeed\":").Append(hasEndpoint ? Number(result.EndSurfaceSpeed) : "null")
                .Append(",\"virtualDeltaV\":").Append(hasEndpoint ? Number(result.DeltaVExpended) : "null")
                .Append(",\"trajectoryStart\":").Append(hasEndpoint && result.Trajectory != null && result.Trajectory.Count > 0 ? Absolute(result.Trajectory[0]) : "null")
                .Append(",\"trajectorySamples\":").Append(hasEndpoint && result.Trajectory != null ? result.Trajectory.Count : 0).Append('}');
            Write(line.ToString());
            }
            catch (Exception) { /* Capture must not affect predictor completion. */ }
        }

        internal static void WorkerException(long id, Exception exception)
        {
            if (id == 0) return;
            Write("{\"schemaVersion\":" + SchemaVersion + ",\"captureSession\":" + Quote(SessionId) + ",\"recordType\":\"worker_exception\",\"submissionId\":" + id +
                  ",\"exceptionType\":" + Quote(exception.GetType().Name) + "}");
        }

        internal static void Abandoned(long id, string reason)
        {
            if (id == 0) return;
            Write("{\"schemaVersion\":" + SchemaVersion + ",\"captureSession\":" + Quote(SessionId) +
                  ",\"recordType\":\"submission_abandoned\",\"submissionId\":" + id +
                  ",\"reason\":" + Quote(reason) + "}");
        }

        internal static void Discarded(long id, string reason)
        {
            if (id == 0) return;
            Write("{\"schemaVersion\":" + SchemaVersion + ",\"captureSession\":" + Quote(SessionId) +
                  ",\"recordType\":\"discarded\",\"submissionId\":" + id +
                  ",\"reason\":" + Quote(reason) + "}");
        }

        internal static void Decision(long id, string decision, long comparedSubmissionId)
        {
            if (id == 0) return;
            Write("{\"schemaVersion\":" + SchemaVersion + ",\"captureSession\":" + Quote(SessionId) +
                  ",\"recordType\":\"selection_decision\",\"submissionId\":" + id +
                  ",\"decision\":" + Quote(decision) +
                  ",\"comparedSubmissionId\":" + comparedSubmissionId + "}");
        }

        // One summary per active transaction. Clearance is deliberately separate
        // from the legacy terrain-contact fields in resolved_result.
        internal static void TargetAwareValidation(long id, TargetAwareAirlessPlanner planner,
            double workerMilliseconds)
        {
            if (id == 0 || planner == null) return;
            try
            {
                var selected = planner.SelectedOutput;
                var terrain = planner.SelectedTerrain;
                var handoff = planner.TerminalHandoff;
                var line = new StringBuilder(700);
                line.Append("{\"schemaVersion\":").Append(SchemaVersion)
                    .Append(",\"captureSession\":").Append(Quote(SessionId))
                    .Append(",\"recordType\":\"target_aware_validation\",\"submissionId\":").Append(id)
                    .Append(",\"processUT\":").Append(Number(Planetarium.GetUniversalTime()))
                    .Append(",\"generation\":").Append(planner.Generation)
                    .Append(",\"sequence\":").Append(planner.Sequence)
                    .Append(",\"stage\":").Append(Quote(planner.Stage.ToString()))
                    .Append(",\"failure\":").Append(Quote(planner.Failure))
                    .Append(",\"directForecast\":").Append(planner.IsDirectForecast ? "true" : "false")
                    .Append(",\"brakeTimeBracketed\":").Append(
                        selected != null && !double.IsNaN(planner.TimingInterval) ? "true" : "false")
                    .Append(",\"ballisticContactUT\":").Append(Number(planner.BallisticContactUT))
                    .Append(",\"ballisticContact\":").Append(planner.BallisticContactUT > 0 ?
                        Absolute(planner.BallisticContact) : "null")
                    .Append(",\"virtualBrakeUT\":").Append(Number(selected?.BrakeUT ?? double.NaN))
                    .Append(",\"signedDownrangeError\":").Append(Number(selected == null ? double.NaN : planner.SignedDownrangeError))
                    .Append(",\"crossrangeError\":").Append(Number(selected == null ? double.NaN : planner.CrossrangeError))
                    .Append(",\"timingInterval\":").Append(Number(selected == null ? double.NaN : planner.TimingInterval))
                    .Append(",\"timingDistanceEstimate\":").Append(Number(selected == null ? double.NaN : planner.TimingDistanceEstimate))
                    .Append(",\"terrainResolved\":").Append(terrain != null && terrain.Resolved ? "true" : "false")
                    .Append(",\"clearPath\":").Append(terrain != null && terrain.ClearPath ? "true" : "false")
                    .Append(",\"minimumSampledClearance\":").Append(Number(terrain?.MinimumSampledClearance ?? double.NaN))
                    .Append(",\"handoffClearance\":").Append(Number(terrain?.HandoffClearance ?? double.NaN))
                    .Append(",\"localTerrainASL\":").Append(Number(terrain?.LocalTerrainASL ?? double.NaN))
                    .Append(",\"endVerticalSpeed\":").Append(Number(selected == null ? double.NaN : handoff.EndVerticalSpeed))
                    .Append(",\"endSurfaceSpeed\":").Append(Number(selected == null ? double.NaN : handoff.EndSurfaceSpeed))
                    .Append(",\"transitionVerticalSpeed\":").Append(Number(selected == null ? double.NaN : handoff.ControllerTransitionVerticalSpeed))
                    .Append(",\"transitionSurfaceSpeed\":").Append(Number(selected == null ? double.NaN : handoff.ControllerTransitionSurfaceSpeed))
                    .Append(",\"optimisticStoppingDistance\":").Append(Number(selected == null ? double.NaN : handoff.OptimisticVerticalStoppingDistance))
                    .Append(",\"terminalNecessaryBoundPasses\":").Append(handoff.NecessaryControlBoundPasses ? "true" : "false")
                    .Append(",\"terrainQueryCount\":").Append(planner.TerrainQueryCount)
                    .Append(",\"terrainQueryElapsedMs\":").Append(Number(planner.TerrainQueryMilliseconds))
                    .Append(",\"workerElapsedMs\":").Append(Number(workerMilliseconds))
                    .Append('}');
                Write(line.ToString());
            }
            catch (Exception) { /* Diagnostic capture cannot affect publication. */ }
        }

        internal static void TargetAwareWorkerStage(long id, string stage, double elapsedMilliseconds,
            int outputCount, string exceptionType)
        {
            if (id == 0) return;
            try
            {
                Write("{\"schemaVersion\":" + SchemaVersion + ",\"captureSession\":" + Quote(SessionId) +
                      ",\"recordType\":\"target_aware_worker_stage\",\"submissionId\":" + id +
                      ",\"processUT\":" + Number(Planetarium.GetUniversalTime()) +
                      ",\"wallTimestamp\":" + Stopwatch.GetTimestamp() +
                      ",\"stage\":" + Quote(stage) +
                      ",\"elapsedMs\":" + Number(elapsedMilliseconds) +
                      ",\"outputCount\":" + outputCount +
                      ",\"exceptionType\":" + Quote(exceptionType) + "}");
            }
            catch (Exception) { /* Diagnostic capture cannot affect worker completion. */ }
        }

        internal static void Lifecycle(string action, long generation, long resultVersion, long activeSubmissionId)
        {
            try
            {
                Write("{\"schemaVersion\":" + SchemaVersion + ",\"captureSession\":" + Quote(SessionId) +
                      ",\"recordType\":\"lifecycle\",\"action\":" + Quote(action) +
                      ",\"processUT\":" + Number(Planetarium.GetUniversalTime()) +
                      ",\"generation\":" + generation + ",\"resultVersion\":" + resultVersion +
                      ",\"activeSubmissionId\":" + activeSubmissionId + "}");
            }
            catch (Exception) { /* Diagnostic lifecycle events cannot alter V1. */ }
        }

        internal static void CaptureError(string stage, Exception exception)
        {
            Write("{\"schemaVersion\":" + SchemaVersion + ",\"captureSession\":" + Quote(SessionId) +
                  ",\"recordType\":\"capture_error\",\"stage\":" + Quote(stage) +
                  ",\"exceptionType\":" + Quote(exception.GetType().Name) + "}");
        }

        // Called after the existing flight-thread terrain resolution. It observes but never changes the result.
        internal static void ResolvedResult(long id, ReentrySimulation.Result result, TerrainProfileTrace terrain,
            string disposition, long currentGeneration, string phase)
        {
            if (id == 0 || result == null) return;
            try
            {
            bool landed = result.Outcome == ReentrySimulation.Outcome.LANDED;
            var line = new StringBuilder(450);
            line.Append("{\"schemaVersion\":").Append(SchemaVersion).Append(",\"captureSession\":").Append(Quote(SessionId))
                .Append(",\"recordType\":\"resolved_result\",\"submissionId\":").Append(id)
                .Append(",\"processUT\":").Append(Number(Planetarium.GetUniversalTime()))
                .Append(",\"wallTimestamp\":").Append(Stopwatch.GetTimestamp())
                .Append(",\"currentGeneration\":").Append(currentGeneration)
                .Append(",\"phase\":").Append(Quote(phase))
                .Append(",\"disposition\":").Append(Quote(disposition))
                .Append(",\"outcome\":").Append(Quote(result.Outcome.ToString()))
                .Append(",\"simulatorLanded\":").Append(landed && result.Trajectory != null ? "true" : "false")
                .Append(",\"terrainContactConfirmed\":").Append(landed && terrain != null && terrain.Applied ? "true" : "false")
                .Append(",\"resolvedEnd\":").Append(landed ? Absolute(result.EndPosition) : "null")
                .Append(",\"resolvedEndASL\":").Append(landed ? Number(result.EndASL) : "null")
                .Append(",\"terrainProfileApplied\":").Append(terrain != null && terrain.Applied ? "true" : "false")
                .Append(",\"terrainProfileStartIndex\":").Append(terrain?.FirstProfileIndex ?? -1)
                .Append(",\"terrainProfileSampleCount\":").Append(terrain?.ProfileSampleCount ?? 0)
                .Append(",\"terrainContactIndex\":").Append(terrain?.ContactIndex ?? -1)
                .Append(",\"terrainContactASL\":").Append(Number(terrain?.ContactTerrainASL ?? double.NaN))
                .Append(",\"terrainQueryCount\":").Append(terrain?.TerrainQueryCount ?? 0)
                .Append(",\"terrainQueryElapsedMs\":").Append(Number(terrain?.TerrainQueryElapsedMs ?? 0))
                .Append(",\"terrainSamples\":").Append(terrain?.CaptureSamples ?? "null")
                .Append('}');
            Write(line.ToString());
            }
            catch (Exception) { /* Capture must not affect V1 result selection. */ }
        }

        internal static void Published(long id, long version, long currentGeneration, string phase)
        {
            if (id == 0) return;
            try
            {
            Write("{\"schemaVersion\":" + SchemaVersion + ",\"captureSession\":" + Quote(SessionId) + ",\"recordType\":\"published\",\"submissionId\":" + id +
                  ",\"processUT\":" + Number(Planetarium.GetUniversalTime()) +
                  ",\"wallTimestamp\":" + Stopwatch.GetTimestamp() +
                  ",\"resultVersion\":" + version +
                  ",\"currentGeneration\":" + currentGeneration +
                  ",\"phase\":" + Quote(phase) + "}");
            }
            catch (Exception) { /* Capture must not affect V1 publication. */ }
        }

        internal static void Close()
        {
            lock (Sync)
            {
                try { _writer?.Dispose(); }
                catch (Exception) { /* Closing capture must not affect landing stop. */ }
                finally
                {
                    _writer = null;
                    _recordsSinceFlush = 0;
                }
            }
        }

        private static void Write(string line)
        {
            try
            {
                lock (Sync)
                {
                    if (_writer == null)
                    {
                        string path = MuUtils.GetCfgPath("LandingGuidanceV1.capture.jsonl");
                        string directory = Path.GetDirectoryName(path);
                        if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
                        _writer = new StreamWriter(new FileStream(path, FileMode.Append, FileAccess.Write, FileShare.Read));
                    }
                    _writer.WriteLine(line);
                    if (++_recordsSinceFlush >= 5)
                    {
                        _writer.Flush();
                        _recordsSinceFlush = 0;
                    }
                }
            }
            catch (Exception ex)
            {
                Trace.WriteLine("[MechJebLandingTrace] passive capture failed: " + ex.Message);
                Close();
            }
        }

        private static string Number(double value) => double.IsNaN(value) || double.IsInfinity(value)
            ? "null" : value.ToString("R", CultureInfo.InvariantCulture);
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        internal static string CaptureNumber(double value) => Number(value);
        private static string Vector(Vector3d value) => "[" + Number(value.x) + "," + Number(value.y) + "," + Number(value.z) + "]";
        private static string Absolute(AbsoluteVector value) => "[" + Number(value.Latitude) + "," + Number(value.Longitude) + "," +
                                                               Number(value.Radius) + "," + Number(value.UT) + "]";
        private static string Quote(string value) => value == null ? "null" : "\"" + value.Replace("\\", "\\\\")
            .Replace("\"", "\\\"").Replace("\r", "\\r").Replace("\n", "\\n") + "\"";
    }
}
