using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace MuMech.Landing
{
    // Worker-owned, bounded, passive observations. Never used by a controller.
    internal sealed class DunaM1Prediction
    {
        internal const int MaxSamples = 128;
        internal readonly string Session;
        internal readonly long Id, Generation;
        internal double IntegrationStartUT = double.NaN, FirstVirtualBrakeUT = double.NaN;
        internal double AcceptedSeconds, LimiterSeconds, VirtualDv, ExcessDv;
        internal int AcceptedSteps, BrakingSteps, ExcessSteps;
        internal double FirstBrakeAcceptedDt = double.NaN, FirstBrakeLimiterDt = double.NaN;
        internal bool SamplesTruncated;
        internal readonly List<Sample> Samples = new List<Sample>();
        private double nextSampleUT = double.NegativeInfinity;
        internal DunaM1Prediction(string session, long id, long generation)
        { Session = session; Id = id; Generation = generation; }

        internal void Start(double ut) => IntegrationStartUT = ut;

        internal void ObserveStep(AbsoluteVector position, double acceptedDt, double limiterDt,
            double cumulativeDv, double maxAccel, double speed, double verticalSpeed,
            double dragAtStepStart, double liftAtStepStart)
        {
            double dv = cumulativeDv - VirtualDv;
            AcceptedSteps++;
            AcceptedSeconds += acceptedDt;
            VirtualDv = cumulativeDv;
            bool firstBrake = dv > 0 && double.IsNaN(FirstVirtualBrakeUT);
            if (dv > 0)
            {
                BrakingSteps++;
                LimiterSeconds += limiterDt;
                double excess = dv - acceptedDt * maxAccel;
                if (excess > 1e-8) { ExcessSteps++; ExcessDv += excess; }
                if (firstBrake)
                {
                    FirstVirtualBrakeUT = position.UT;
                    FirstBrakeAcceptedDt = acceptedDt;
                    FirstBrakeLimiterDt = limiterDt;
                }
            }
            if (!firstBrake && position.UT < nextSampleUT) return;
            if (Samples.Count >= MaxSamples)
            {
                SamplesTruncated = true;
                return;
            }
            Samples.Add(new Sample(position, acceptedDt, limiterDt, dv, cumulativeDv,
                speed, verticalSpeed, dragAtStepStart, liftAtStepStart));
            nextSampleUT = position.UT + 5;
        }

        internal readonly struct Sample
        {
            internal readonly AbsoluteVector Position;
            internal readonly double AcceptedDt, LimiterDt, Dv, CumulativeDv, Speed, VerticalSpeed, Drag, Lift;
            internal Sample(AbsoluteVector position, double acceptedDt, double limiterDt, double dv,
                double cumulativeDv, double speed, double verticalSpeed, double drag, double lift)
            {
                Position = position; AcceptedDt = acceptedDt; LimiterDt = limiterDt;
                Dv = dv; CumulativeDv = cumulativeDv; Speed = speed; VerticalSpeed = verticalSpeed;
                Drag = drag; Lift = lift;
            }
        }

        // Ordinary LANDED is only a radial crossing. M1 never validates touchdown.
        internal static string ContactMeaning(ReentrySimulation.Outcome outcome) =>
            outcome == ReentrySimulation.Outcome.LANDED ? "geometric_contact_only" :
            outcome == ReentrySimulation.Outcome.IMPACT ? "impact" : "no_geometric_contact";
    }

    // Flight-thread writer. One explicitly armed capture per process; never append to
    // the legacy trace. No worker I/O, no terrain queries, no change to result ownership.
    internal sealed class DunaM1Capture
    {
        internal const int MaxRecords = 20000;
        internal const long MaxBytes = 128L * 1024 * 1024;
        private bool runtimeModelAttempted;
        private readonly string forceProbePhase;
        private static DunaM1DiagnosticsRequest request;
        private bool forceProbeWarpWaitLogged;
        private bool forceProbeAttempted;
        private DunaM1ForceProbe forceProbe;
        private double forceProbeStartUT, forceProbeLastUT;
        private double nextPartObservationUT = double.NegativeInfinity;
        private readonly string session = Guid.NewGuid().ToString("N");
        private readonly Guid vessel;
        private readonly double latitude, longitude;
        private StreamWriter writer;
        private int records;
        private long bytes, submission;
        private double nextLive = double.NegativeInfinity, nextPrediction = double.NegativeInfinity, lastLive;
        private string lastPhase;
        internal bool Open => writer != null;

        private DunaM1Capture(Guid vessel, double latitude, double longitude, double ut, double touchdown,
            string forcePhase, string receipt)
        {
            this.vessel = vessel; this.latitude = latitude; this.longitude = longitude; lastLive = ut;
            forceProbePhase = forcePhase;
            string path = MuUtils.GetCfgPath("DunaM1-" + session + ".jsonl");
            string directory = Path.GetDirectoryName(path);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            writer = new StreamWriter(new FileStream(path, FileMode.CreateNew, FileAccess.Write, FileShare.Read),
                new UTF8Encoding(false));
            Write("session_start", ",\"ut\":" + N(ut) + ",\"touchdownSetpoint\":" + N(touchdown) +
                ",\"maxRecords\":" + MaxRecords + ",\"maxBytes\":" + MaxBytes +
                ",\"runtimeObservationRevision\":4,\"thrustVectorMeaning\":\"neutral_projection_not_actual\"" +
                ",\"activationSource\":\"one_shot_config\",\"requestReceipt\":" + Q(Path.GetFileName(receipt)) +
                ",\"forceProbeRequestedPhase\":" + Q(forceProbePhase) +
                ",\"assemblyVersion\":" + Q(DunaM1RuntimeObservation.BuildVersion()) +
                ",\"assemblyMvid\":" + Q(typeof(DunaM1Capture).Assembly.ManifestModule.ModuleVersionId.ToString("D")));
            if (Open) Debug.Log("[DunaM1] capture started: " + path + "; forcePhase=" + (forceProbePhase ?? "disabled"));
        }

        internal static void Initialize()
        {
            if (request != null) return;
            string path = MuUtils.GetCfgPath(DunaM1DiagnosticsRequest.FileName);
            request = DunaM1DiagnosticsRequest.Load(path);
            Debug.Log("[DunaM1] request " + request.Status + "; forcePhase=" +
                (request.ForcePhase ?? "disabled") + "; path=" + path);
        }

        internal static bool Armed => request != null && request.Armed;

        internal static DunaM1Capture TryOpen(Guid vessel, double latitude, double longitude, double ut, double touchdown)
        {
            if (!Armed) return null;
            if (!request.TryConsume(out string receipt, out string reason))
            {
                Debug.Log("[DunaM1] activation rejected: " + reason);
                return null;
            }
            Debug.Log("[DunaM1] request consumed: " + receipt);
            var capture = new DunaM1Capture(vessel, latitude, longitude, ut, touchdown, request.ForcePhase, receipt);
            return capture.Open ? capture : null;
        }

        internal bool Matches(Guid id, double lat, double lon, double ut) =>
            id == vessel && lat == latitude && lon == longitude && ut >= lastLive;

        internal bool LiveDue(double ut, string phase) => Open &&
            (phase != lastPhase || ut >= nextLive);

        internal DunaM1Prediction Submit(Orbit patch, VesselState state, long generation,
            string policy, double decelASL, double probableASL, double dt)
        {
            if (!Open || state.Time < nextPrediction) return null;
            nextPrediction = state.Time + 1;
            var p = new DunaM1Prediction(session, ++submission, generation);
            Write("prediction_submit", Id(p) + ",\"captureUT\":" + N(state.Time) +
                ",\"inputUT\":" + N(patch.StartUT) + ",\"positionBCI\":" + V(patch.WorldBCIPositionAtUT(patch.StartUT)) +
                ",\"velocityBCI\":" + V(patch.WorldOrbitalVelocityAtUT(patch.StartUT)) +
                ",\"mass\":" + N(state.Mass) + ",\"maximumThrustAcceleration\":" + N(state.LimitedMaxThrustAcceleration) +
                ",\"policy\":" + Q(policy) + ",\"decelEndASL\":" + N(decelASL) +
                ",\"probableTerrainASL\":" + N(probableASL) + ",\"inputDt\":" + N(dt));
            return Open ? p : null;
        }

        internal void Result(ReentrySimulation.Result r, string decision, double ut, long version)
        {
            try
            {
                var p = r.DunaM1;
                if (!Open || p == null || p.Session != session) return;
                bool contact = r.Outcome == ReentrySimulation.Outcome.LANDED;
                bool hasEndpoint = r.Outcome != ReentrySimulation.Outcome.ERROR &&
                    r.Outcome != ReentrySimulation.Outcome.NO_REENTRY && r.Body != null;
                var b = new StringBuilder(Id(p));
                b.Append(",\"processUT\":").Append(N(ut)).Append(",\"versionBeforeDecision\":").Append(version);
                b.Append(",\"decision\":").Append(Q(decision)).Append(",\"outcome\":").Append(Q(r.Outcome.ToString()));
                b.Append(",\"contactMeaning\":").Append(Q(DunaM1Prediction.ContactMeaning(r.Outcome)));
                b.Append(",\"safeTouchdownValidated\":false,\"integrationStartUT\":").Append(N(p.IntegrationStartUT));
                b.Append(",\"firstVirtualBrakeUT\":").Append(N(p.FirstVirtualBrakeUT));
                b.Append(",\"firstBrakeAcceptedDt\":").Append(N(p.FirstBrakeAcceptedDt));
                b.Append(",\"firstBrakeLimiterDt\":").Append(N(p.FirstBrakeLimiterDt));
                b.Append(",\"acceptedSteps\":").Append(p.AcceptedSteps).Append(",\"acceptedSeconds\":").Append(N(p.AcceptedSeconds));
                b.Append(",\"brakingSteps\":").Append(p.BrakingSteps).Append(",\"limiterSeconds\":").Append(N(p.LimiterSeconds));
                b.Append(",\"virtualDv\":").Append(N(p.VirtualDv)).Append(",\"excessSteps\":").Append(p.ExcessSteps);
                b.Append(",\"excessDv\":").Append(N(p.ExcessDv));
                b.Append(",\"endUT\":").Append(hasEndpoint ? N(r.EndUT) : "null");
                b.Append(",\"endpoint\":").Append(hasEndpoint ? A(r.EndPosition) : "null");
                b.Append(",\"endSurfaceSpeed\":").Append(hasEndpoint ? N(r.EndSurfaceSpeed) : "null");
                b.Append(",\"contactSurfaceSpeed\":").Append(contact && hasEndpoint ? N(r.EndSurfaceSpeed) : "null");
                b.Append(",\"probableTerrainASL\":").Append(N(r.InputProbableLandingSiteASL));
                b.Append(",\"resolvedEndpointTerrainASL\":").Append(hasEndpoint ? N(r.EndASL) : "null");
                b.Append(",\"endpointTerrainClearance\":").Append(hasEndpoint ? N(r.EndPosition.Radius-r.Body.Radius-r.EndASL) : "null");
                b.Append(",\"samplesTruncated\":").Append(p.SamplesTruncated ? "true" : "false");
                b.Append(",\"samples\":[");
                for (int i = 0; i < p.Samples.Count; i++)
                {
                    if (i != 0) b.Append(',');
                    var a = p.Samples[i];
                    b.Append('[').Append(A(a.Position)).Append(',').Append(N(a.AcceptedDt)).Append(',')
                        .Append(N(a.LimiterDt)).Append(',').Append(N(a.Dv)).Append(',').Append(N(a.CumulativeDv))
                        .Append(',').Append(N(a.Speed)).Append(',').Append(N(a.VerticalSpeed)).Append(',')
                        .Append(N(a.Drag)).Append(',').Append(N(a.Lift)).Append(']');
                }
                b.Append(']');
                Write("prediction_result", b.ToString());
            }
            catch (Exception ex) { Fail(ex); }
        }

        internal void Published(ReentrySimulation.Result r, double ut, long version)
        {
            try
            {
                if (!Open) return;
                Write("prediction_published", ",\"ut\":" + N(ut) + ",\"version\":" + version +
                    ",\"inputUT\":" + N(r.InputUT) + ",\"predictionId\":" + PredictionId(r));
            }
            catch (Exception ex) { Fail(ex); }
        }

        internal void Live(Vessel v, VesselState s, string phase, string status, string detail,
            long version, ReentrySimulation.Result prediction, double throttle, double attitudeError,
            string thrustMode, double speedSetpoint, bool killHorizontal, string terminalReason,
            MechJebModuleThrustController thrust, MechJebModuleAttitudeController attitude)
        {
            lastLive = s.Time; lastPhase = phase;
            nextLive = s.Time + (phase == "CourseCorrection" || phase == "DecelerationBurn" ||
                phase == "KillHorizontalVelocity" || phase == "FinalDescent" ? .1 : 1);
            // Raw observations at this callback, not a verified post-physics boundary.
            bool includeParts = !runtimeModelAttempted || s.Time >= nextPartObservationUT || terminalReason != null;
            string runtime = DunaM1RuntimeObservation.Read(v, s, !runtimeModelAttempted, includeParts, thrust, attitude);
            if (includeParts) nextPartObservationUT = s.Time + 1;
            runtimeModelAttempted = true;
            Write("live", ",\"ut\":" + N(s.Time) + ",\"phase\":" + Q(phase) +
                ",\"status\":" + Q(status) + ",\"controllerDetail\":" + Q(detail) +
                ",\"terminalReason\":" + Q(terminalReason) + ",\"situation\":" + Q(v.situation.ToString()) +
                ",\"latitude\":" + N(v.latitude) + ",\"longitude\":" + N(v.longitude) +
                ",\"altitudeASL\":" + N(s.AltitudeASL) + ",\"altitudeTrue\":" + N(s.AltitudeTrue) +
                ",\"altitudeBottom\":" + N(s.AltitudeBottom) + ",\"mass\":" + N(s.Mass) +
                ",\"surfaceSpeed\":" + N(s.SpeedSurface) + ",\"verticalSpeed\":" + N(s.SpeedVertical) +
                ",\"horizontalSpeed\":" + N(s.SpeedSurfaceHorizontal) + ",\"surfaceVelocityNEU\":" + NEU(s.SurfaceVelocity,s) +
                ",\"forwardNEU\":" + NEU(s.Forward,s) + ",\"thrustVectorNEU\":null" +
                ",\"neutralThrustVectorNEU\":" + NEU(s.ThrustVectorLastFrame,s) +
                ",\"runtimeObservation\":" + runtime +
                ",\"dragAccelNEU\":" + NEU(s.PureDragVector,s) + ",\"liftAccelNEU\":" + NEU(s.PureLiftVector,s) +
                ",\"dragAcceleration\":" + N(s.DragAcceleration) + ",\"liftAcceleration\":" + N(s.LiftAcceleration) +
                ",\"density\":" + N(s.AtmosphericDensity) + ",\"pressureKPa\":" + N(v.staticPressurekPa) +
                ",\"dynamicPressurePa\":" + N(s.DynamicPressure) + ",\"mach\":" + N(s.Mach) +
                ",\"commandedThrottle\":" + N(throttle) + ",\"appliedThrottle\":" + N(v.ctrlState.mainThrottle) +
                ",\"thrustAcceleration\":" + N(s.CurrentThrustAcceleration) + ",\"limitedMaxThrustAcceleration\":" + N(s.LimitedMaxThrustAcceleration) +
                ",\"engineResponseSeconds\":" + N(s.MaxEngineResponseTime) + ",\"maximumMassFlow\":" + N(s.MaximumEngineMassFlow) +
                ",\"localGravity\":" + N(s.LocalGravity) + ",\"attitudeErrorDegrees\":" + N(attitudeError) +
                ",\"physicsDt\":" + N(s.DeltaT) + ",\"warpRate\":" + N(TimeWarp.CurrentRate) +
                ",\"thrustMode\":" + Q(thrustMode) + ",\"speedSetpoint\":" + N(speedSetpoint) +
                ",\"killHorizontal\":" + (killHorizontal ? "true" : "false") +
                ",\"predictionVersion\":" + version + ",\"predictionId\":" + PredictionId(prediction) +
                ",\"predictionInputUT\":" + N(prediction?.InputUT ?? double.NaN) +
                ",\"predictionEndpoint\":" + (prediction != null && prediction.Outcome == ReentrySimulation.Outcome.LANDED ? A(prediction.EndPosition) : "null"));
        }

        // Observe every eligible landing OnFixedUpdate callback during a separately armed short
        // calibration burst, not only the slower ordinary live sampling cadence.
        internal void ObserveForceProbe(Vessel v, VesselState s, string phase)
        {
            if (!Open || string.IsNullOrEmpty(forceProbePhase)) return;
            if (!forceProbeAttempted && phase == forceProbePhase && TimeWarp.CurrentRate != 1 &&
                !forceProbeWarpWaitLogged)
            {
                forceProbeWarpWaitLogged = true;
                Debug.Log("[DunaM1] force probe waiting: requested phase reached, warp not 1x");
            }
            if (!forceProbeAttempted && phase == forceProbePhase && TimeWarp.CurrentRate == 1)
            {
                forceProbeAttempted = true;
                forceProbeStartUT = forceProbeLastUT = s.Time;
                forceProbe = DunaM1ForceProbe.Start(v.id, out string status);
                Write("force_probe_start", ",\"ut\":" + N(s.Time) + ",\"phase\":" + Q(phase) + ",\"probe\":" + status);
                Debug.Log("[DunaM1] force probe start: " + status);
            }
            DunaM1ForceProbe p = forceProbe;
            if (!Open || p == null) return;
            string reason = s.Time < forceProbeLastUT ? "state_ut_regressed" :
                TimeWarp.CurrentRate != 1 ? "warp_changed" :
                phase != forceProbePhase ? "phase_changed" :
                p.LimitReached ? "event_limit" : null;
            if (reason != null) { FinishForceProbe(reason, s.Time); return; }
            p.ObserveState(v, s, phase);
            forceProbeLastUT = s.Time;
            Write("force_probe_batch", ",\"ut\":" + N(s.Time) + ",\"batch\":" + p.Drain());
            if (p.WindowStopReason != null || s.Time - forceProbeStartUT >= DunaM1ForceProbe.WindowSeconds)
                FinishForceProbe(p.WindowStopReason ?? "window_complete", s.Time);
        }

        private void FinishForceProbe(string reason, double ut)
        {
            DunaM1ForceProbe p = forceProbe;
            forceProbe = null;
            if (p == null) return;
            bool clean = p.Stop();
            Write("force_probe_batch", ",\"ut\":" + N(ut) + ",\"batch\":" + p.Drain());
            Write("force_probe_end", ",\"ut\":" + N(ut) + ",\"reason\":" + Q(reason) +
                ",\"cleanupOk\":" + (clean ? "true" : "false"));
            Debug.Log("[DunaM1] force probe ended: " + reason + "; unpatchCallsOk=" + clean);
        }

        private string PredictionId(ReentrySimulation.Result result) =>
            result?.DunaM1 != null && result.DunaM1.Session == session ?
                result.DunaM1.Id.ToString(CultureInfo.InvariantCulture) : "null";

        internal void Close(string reason)
        {
            if (!Open) return;
            try
            {
                FinishForceProbe(reason, lastLive);
                Write("session_end", ",\"reason\":" + Q(reason) + ",\"forceProbeAttempted\":" +
                    (forceProbeAttempted ? "true" : "false"));
                Debug.Log("[DunaM1] capture ended: " + reason + "; forceProbeAttempted=" + forceProbeAttempted);
            }
            finally { DisposeWriter(); }
        }

        internal void Fail(Exception ex)
        {
            DisposeWriter();
            Debug.Log("[DunaM1] capture stopped: " + ex.Message);
        }

        private void DisposeWriter()
        {
            var w = writer; writer = null;
            DunaM1ForceProbe p = forceProbe; forceProbe = null;
            if (p != null && !p.Stop()) Debug.Log("[DunaM1] force probe cleanup incomplete; observer disabled");
            try { w?.Dispose(); } catch (Exception) { /* Diagnostics must not affect flight. */ }
        }

        private void Write(string kind, string fields)
        {
            if (!Open) return;
            try
            {
                string line = "{\"schemaVersion\":1,\"session\":" + Q(session) + ",\"recordType\":" + Q(kind) +
                    ",\"body\":\"Duna\",\"vessel\":" + Q(vessel.ToString()) + ",\"targetLat\":" + N(latitude) +
                    ",\"targetLon\":" + N(longitude) + fields + "}\n";
                int size = Encoding.UTF8.GetByteCount(line);
                if (records >= MaxRecords || bytes + size > MaxBytes)
                {
                    DisposeWriter();
                    Debug.Log("[DunaM1] capture limit reached");
                    return;
                }
                writer.Write(line);
                records++; bytes += size;
                if (records % 10 == 0 || kind == "session_start" || kind == "session_end" ||
                    kind == "force_probe_start" || kind == "force_probe_end") writer.Flush();
            }
            catch (Exception ex) { Fail(ex); }
        }

        private static string Id(DunaM1Prediction p) => ",\"predictionId\":" + p.Id + ",\"generation\":" + p.Generation;
        internal static string N(double n) => double.IsNaN(n) || double.IsInfinity(n) ? "null" : n.ToString("R", CultureInfo.InvariantCulture);
        private static string A(AbsoluteVector a) => "[" + N(a.Latitude) + "," + N(a.Longitude) + "," + N(a.Radius) + "," + N(a.UT) + "]";
        private static string V(Vector3d a) => "[" + N(a.x) + "," + N(a.y) + "," + N(a.z) + "]";
        private static string NEU(Vector3d a, VesselState s) => "[" + N(Vector3d.Dot(a,s.North)) + "," +
            N(Vector3d.Dot(a,s.East)) + "," + N(Vector3d.Dot(a,s.Up)) + "]";
        internal static string Q(string value)
        {
            if (value == null) return "null";
            var b = new StringBuilder("\"");
            foreach (char c in value)
            {
                if (c == '"' || c == '\\') b.Append('\\').Append(c);
                else if (c < 32) b.Append("\\u").Append(((int)c).ToString("x4", CultureInfo.InvariantCulture));
                else b.Append(c);
            }
            return b.Append('"').ToString();
        }
    }
}
