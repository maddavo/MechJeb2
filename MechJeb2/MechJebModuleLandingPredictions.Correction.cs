using System;
using System.Threading;
using MuMech.Landing;

namespace MuMech
{
    public partial class MechJebModuleLandingPredictions
    {
        private AirlessTargetAwareSnapshot? correctionSourceSnapshot;
        private CorrectionTrial correctionTrial;

        private sealed class CorrectionTrial
        {
            internal readonly long Generation;
            internal readonly object Body;
            internal readonly double Latitude, Longitude, RequestedUT, StartUT, MaximumThrust, MaximumFlow, MaximumCommand, Reference, PolicyTerrain;
            internal readonly Vector3d Direction;
            internal volatile bool Cancelled, WorkerDone;
            internal CorrectionBurnPlan Plan;
            internal TargetAwareAirlessPlanner Planner;
            internal TargetAwareTerminalEnvelope Terminal;
            internal string Failure;
            internal CorrectionTrial(AirlessTargetAwareSnapshot s, long generation, object body,
                Vector3d direction, double now, double start, double reference)
            {
                Generation = generation; Body = body; Latitude = s.TargetLatitude; Longitude = s.TargetLongitude;
                Direction = direction; RequestedUT = now; StartUT = start; Reference = reference;
                MaximumThrust = s.MaximumThrust; MaximumFlow = s.MaximumMassFlow; MaximumCommand = s.MaximumCommandThrottle;
                PolicyTerrain = s.DecelEndASL;
            }
        }

        private void CancelCorrectionTrial()
        {
            if (correctionTrial == null) return;
            correctionTrial.Cancelled = true;
            correctionTrial.Planner?.CancelWorkers();
            correctionTrial = null;
        }

        // 0: conservative fallback; 1: pending; 2: independently validated plan.
        internal int PrepareCorrectionBurn(Vector3d direction, double requested, double conservative,
            double preparationSeconds, double deadline, out CorrectionBurnPlan plan, out string reason)
        {
            plan = null; reason = "coarse_unavailable";
            double now = VesselState.Time;
            if (correctionTrial != null)
            {
                var t = correctionTrial;
                if (!CorrectionTrialCompatible(t) || Vector3d.Angle(t.Direction, direction) > 20 ||
                    now >= t.StartUT)
                { CancelCorrectionTrial(); reason = "coarse_context_or_deadline"; return 0; }
                if (t.Failure != null) { reason = t.Failure; CancelCorrectionTrial(); return 0; }
                if (t.Planner?.TerminalTouchdownValidated == true)
                {
                    double error = CorrectionBurnPlanner.Error(t.Plan.AfterBurn, t.Planner.NominalTerminalTerrain.Contact);
                    if (error >= t.Plan.BaselineError) { CancelCorrectionTrial(); reason = "coarse_no_terrain_validated_improvement"; return 0; }
                    var first = t.Plan.Forecast.FirstRequiredThrust;
                    double turn = first.HasValue ? Core.Landing.WarpPreparation.Estimate(Vector3d.Angle(t.Direction,
                        -(first.Value.Velocity - Vector3d.Cross(t.Plan.AfterBurn.AngularVelocity, first.Value.Position)))) : double.NaN;
                    if (!first.HasValue || double.IsNaN(turn) || double.IsInfinity(turn) ||
                        t.Plan.AfterBurn.StateUT + turn + .75 + 2 * TargetAwareRefreshSeconds + VesselState.DeltaT >= first.Value.UT)
                    { CancelCorrectionTrial(); reason = "coarse_brake_preparation_margin"; return 0; }
                    plan = t.Plan; reason = "coarse_finite_burn_validated";
                    correctionTrial = null; // Transfer the immutable plan, never its trial forecast ownership.
                    return 2;
                }
                reason = "coarse_validating"; return 1;
            }
            if (!correctionSourceSnapshot.HasValue) return 0;
            var s = correctionSourceSnapshot.Value;
            if (Result == null || now - s.InputUT > 2 * TargetAwareRefreshSeconds ||
                s.MinimumCommandThrottle != 0 || s.MinimumThrust != 0 || s.ThrottleSmoothingSeconds != 0 ||
                VesselState.MaxEngineResponseTime != 0 || Vessel.ctrlState.mainThrottle != 0 ||
                VesselState.CurrentThrustAcceleration != 0 || !(requested > conservative * 2) ||
                double.IsNaN(preparationSeconds) || double.IsInfinity(preparationSeconds)) return 0;
            double start = now + preparationSeconds + 2 * TargetAwareRefreshSeconds + VesselState.DeltaT;
            double reference = Result.BrakeReferenceUT(double.NaN);
            if (!(start < reference - 5 && start < deadline)) return 0;
            var trial = new CorrectionTrial(s, Interlocked.Read(ref predictionGeneration), MainBody,
                direction.normalized, now, start, reference);
            correctionTrial = trial;
            if (!ThreadPool.QueueUserWorkItem(_ =>
            {
                try
                {
                    trial.Plan = CorrectionBurnPlanner.Search(s, trial.Direction, requested, conservative,
                        start, reference, deadline, () => trial.Cancelled || predictorDestroyed);
                    if (trial.Plan == null) trial.Failure = "coarse_no_better_finite_burn";
                }
                catch (Exception ex) { trial.Failure = "coarse_worker_" + ex.GetType().Name; }
                finally { trial.WorkerDone = true; }
            })) { CancelCorrectionTrial(); return 0; }
            reason = "coarse_searching"; return 1;
        }

        private bool CorrectionTrialCompatible(CorrectionTrial t) => !predictorDestroyed && !t.Cancelled &&
            Core.Landing.CurrentStep is CourseCorrection && t.Generation == Interlocked.Read(ref predictionGeneration) &&
            ReferenceEquals(t.Body, MainBody) && t.Latitude == (double)Core.Target.targetLatitude &&
            t.Longitude == (double)Core.Target.targetLongitude &&
            CorrectionBurnPlanner.CapabilityCompatible(VesselState.ThrustVectorMaxThrottle.magnitude, t.MaximumThrust,
                VesselState.MaximumEngineMassFlow, t.MaximumFlow) &&
            Core.Thrust.ThrottleFixedLimit == t.MaximumCommand && !Core.Thrust.LimiterMinThrottle &&
            VesselState.MaxEngineResponseTime == 0 && Result != null &&
            Math.Abs(Result.BrakeReferenceUT(double.NaN) - t.Reference) <= VesselState.DeltaT &&
            Result.InputDecelEndAltitudeASL == t.PolicyTerrain;

        private void AdvanceCorrectionTrial(bool terrainAvailable)
        {
            var t = correctionTrial;
            if (t == null) return;
            // Plan may still be null while the copied-data worker is running.
            if (predictorDestroyed || !(Core.Landing.CurrentStep is CourseCorrection) ||
                t.Generation != Interlocked.Read(ref predictionGeneration) || !ReferenceEquals(t.Body, MainBody) ||
                t.Latitude != (double)Core.Target.targetLatitude || t.Longitude != (double)Core.Target.targetLongitude)
            { CancelCorrectionTrial(); return; }
            if (VesselState.Time >= t.StartUT) { t.Failure = "coarse_validation_deadline"; return; }
            if (!t.WorkerDone || t.Failure != null || t.Plan == null) return;
            // Active current-state forecast always has priority. Never double
            // the flight thread's terrain budget by advancing both planners.
            if (!terrainAvailable || activeTargetAware != null) return;
            try
            {
                if (t.Planner == null)
                {
                    var body = (CelestialBody)t.Body;
                    t.Planner = new TargetAwareAirlessPlanner(t.Plan.AfterBurn, t.Generation, 0, body,
                        (lat, lon) => body.TerrainAltitude(lat, lon), 200, 512, 1536,
                        directForecast: true, fixedBrakeReferenceUT: t.Reference);
                    t.Planner.SetDirectOutput(t.Plan.Forecast);
                }
                if (t.Terminal != null)
                { t.Planner.SetTerminalEnvelope(t.Terminal); t.Terminal = null; }
                if (t.Planner.Stage == TargetAwarePlannerStage.ReadyTerminal)
                {
                    t.Planner.BeginTerminalWorker(); t.WorkerDone = false;
                    if (!ThreadPool.QueueUserWorkItem(_ =>
                    {
                        try { t.Terminal = t.Planner.RunTerminalEnvelope(); }
                        catch (Exception ex) { t.Failure = "coarse_terminal_" + ex.GetType().Name; }
                        finally { t.WorkerDone = true; }
                    })) { t.WorkerDone = true; t.Failure = "coarse_terminal_queue_rejected"; }
                    return;
                }
                if (t.Planner.Stage != TargetAwarePlannerStage.AwaitTerminal)
                    AdvanceTargetAwareTerrain(t.Planner);
                if (t.Planner.Stage == TargetAwarePlannerStage.Failed || t.Planner.Stage == TargetAwarePlannerStage.Uncertain)
                    t.Failure = "coarse_terrain_" + t.Planner.Failure;
                if (t.Planner.Stage == TargetAwarePlannerStage.Complete && !t.Planner.TerminalTouchdownValidated)
                    t.Failure = "coarse_not_landable";
            }
            catch (Exception ex) { t.Failure = "coarse_dispatch_" + ex.GetType().Name; }
        }
    }
}
