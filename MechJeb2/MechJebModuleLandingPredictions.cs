extern alias JetBrainsAnnotations;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Threading;
using Smooth.Dispose;
using UnityEngine;
using UnityToolbag;
using MuMech.Landing;
using Debug = UnityEngine.Debug;
using Random = System.Random;

namespace MuMech
{
    public partial class MechJebModuleLandingPredictions : ComputerModule
    {
        private readonly TargetAwareHeadingObservation terminalHeading = new TargetAwareHeadingObservation();
        // TODO Move the endASL code to the CheckResult method
        // For Compatibility with RPM
        public ReentrySimulation.Result GetResult() => Result;

        public ReentrySimulation.Result Result =>
            //if (result != null)
            //{
            //    if (result.body != null)
            //    {
            //        simDragScalar = result.prediction.firstDrag;
            //        simLiftScalar = result.prediction.firstLift;
            //        simDynamicPressurePa = result.prediction.dynamicPressurekPa * 1000;
            //        simMach = result.prediction.mach;
            //        simSpeedOfSound = result.prediction.speedOfSound;
            //
            //        if (result.debugLog != "")
            //        {
            //
            //            MechJebCore.print("Now".PadLeft(8)
            //                       + " Alt:" + vesselState.altitudeASL.ToString("F0").PadLeft(6)
            //                       + " Vel:" + vesselState.speedOrbital.ToString("F2").PadLeft(8)
            //                       + " AirVel:" + vesselState.speedSurface.ToString("F2").PadLeft(8)
            //                       + " SoS:" +  vesselState.speedOfSound.ToString("F2").PadLeft(6)
            //                       + " mach:" + vesselState.mach.ToString("F2").PadLeft(6)
            //                       + " dynP:" + (vesselState.dynamicPressure / 1000).ToString("F5").PadLeft(9)
            //                       + " Temp:" + vessel.atmosphericTemperature.ToString("F2").PadLeft(8)
            //                       + " Lat:" + vesselState.latitude.ToString("F2").PadLeft(6));
            //
            //
            //            MechJebCore.print(result.debugLog);
            //            result.debugLog = "";
            //
            //            Vector3 scaledPos = ScaledSpace.LocalToScaledSpace(vessel.transform.position);
            //            Vector3 sunVector = (FlightGlobals.Bodies[0].scaledBody.transform.position - scaledPos).normalized;
            //
            //            float sunDot = Vector3.Dot(sunVector, vessel.upAxis);
            //            float sunAxialDot = Vector3.Dot(sunVector, vessel.mainBody.bodyTransform.up);
            //            MechJebCore.print("sunDot " + sunDot.ToString("F3") + " sunAxialDot " + sunAxialDot.ToString("F3") + " " + PhysicsGlobals.DragUsesAcceleration);
            //
            //        }
            //    }
            //}
            predictorDestroyed ? null : TargetAwareAirlessActive &&
            Core.Landing.CurrentStep is CourseCorrection correction &&
            result != null && !correction.PublishedPredictionUsable(result.InputUT)
                ? null : AvailableForecast();

        private readonly PublishedForecastHold retainedCorrectionForecast = new PublishedForecastHold();
        private void ClearRetainedCorrectionForecast()
        {
            retainedCorrectionForecast.Dispose();
        }
        // This is the same published payload, with control eligibility revoked.
        // It has one owner until replacement/context reset, never a second model.
        internal ReentrySimulation.Result PresentedForecast
        {
            get
            {
                var current = Result;
                if (current != null && current.Outcome != ReentrySimulation.Outcome.NO_REENTRY) return current;
                if (impactDiagnostic != null && impactLineage.HasValue &&
                    ReferenceEquals(impactDiagnostic.Body, Core.Target.targetBody) &&
                    impactLineage.Value.TargetLatitude == (double)Core.Target.targetLatitude &&
                    impactLineage.Value.TargetLongitude == (double)Core.Target.targetLongitude) return impactDiagnostic;
                var retained = retainedCorrectionForecast.Observe(Core.Target.PositionTargetExists ? Core.Target.targetBody : null,
                    Core.Target.targetLatitude, Core.Target.targetLongitude);
                if (retained != null) return retained;
                if (TargetAwareAirlessActive && result != null && committedTargetAware.HasValue &&
                    ReferenceEquals(result.Body, Core.Target.targetBody) &&
                    committedTargetAware.Value.TargetLatitude == (double)Core.Target.targetLatitude &&
                    committedTargetAware.Value.TargetLongitude == (double)Core.Target.targetLongitude) return result;
                return null;
            }
        }
        internal bool PresentedForecastCurrent => PresentedForecast != null && ReferenceEquals(PresentedForecast, Result) &&
            !(Core.Landing.CurrentStep is CourseCorrection c && c.ForecastChanging);
        internal string PresentedForecastState => PresentedForecast == null ? "Forecast: updating" :
            PresentedForecastCurrent ? "Forecast: current" : "Previous forecast: updating";

        private ReentrySimulation.Result AvailableForecast()
        {
            if (!TargetAwareAirlessActive || !committedTargetAware.HasValue && !impactLineage.HasValue)
                return result;
            if (publishedBrakingCapabilityEpoch != sustainedBraking.Epoch)
                return Core.Landing.CurrentStep is DecelerationBurn ? unavailableTimingView : null;
            // A newer impact permanently contradicts the older landing claim,
            // even after that impact itself expires. Only a newer landing can
            // restore readiness. No failed worker changes this observation.
            TargetAwareResultLineage latest = TargetAwareForecastPresentation.Latest(
                committedTargetAware, impactLineage).Value;
            ReentrySimulation.LandingForecastKind kind = TargetAwareForecastPresentation.AvailableKind(
                latest, Interlocked.Read(ref predictionGeneration), Core.Target.targetBody,
                Core.Target.targetLatitude, Core.Target.targetLongitude, latest.TargetTerrainASL,
                Planetarium.GetUniversalTime(), TargetAwareMaximumSnapshotAgeSeconds);
            if (kind == ReentrySimulation.LandingForecastKind.ImpactForecast) return impactDiagnostic;
            if (kind == ReentrySimulation.LandingForecastKind.LandableForecast) return result;
            return Core.Landing.CurrentStep is DecelerationBurn ? unavailableTimingView : null;
        }

        private void ClearImpactDiagnostic()
        {
            impactDiagnostic?.Release();
            impactDiagnostic = null;
            impactLineage = null;
            unavailableTimingView = null;
        }

        public ReentrySimulation.Result GetErrorResult()
        {
            if (null != errorResult)
            {
                if (null != errorResult.Body)
                {
                    errorResult.EndASL = errorResult.Body.TerrainAltitude(errorResult.EndPosition.Latitude, errorResult.EndPosition.Longitude);
                }
            }

            return errorResult;
        }

        [ValueInfoItem("#MechJeb_SimDragScalar", InfoItem.Category.Vessel, format = ValueInfoItem.SI, units = "m/s²")] //Sim Drag Scalar
        public double simDragScalar;

        [ValueInfoItem("#MechJeb_SimLiftScalar", InfoItem.Category.Vessel, format = ValueInfoItem.SI, units = "m/s²")] //Sim Lift Scalar
        public double simLiftScalar;

        [ValueInfoItem("#MechJeb_SimDynaPressPa", InfoItem.Category.Vessel, format = ValueInfoItem.SI, units = "Pa")] //Sim DynaPressPa
        public double simDynamicPressurePa;

        [ValueInfoItem("#MechJeb_SimsimMach", InfoItem.Category.Vessel, format = "F2")] //Sim simMach
        public double simMach;

        [ValueInfoItem("#MechJeb_SimSpdOfSnd", InfoItem.Category.Vessel, format = ValueInfoItem.SI, units = "m/s")] //Sim SpdOfSnd
        public double simSpeedOfSound;

        //inputs:
        [Persistent(pass = (int)Pass.GLOBAL)]
        public bool makeAerobrakeNodes = false;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public bool showTrajectory = false;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public bool worldTrajectory = true;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public bool camTrajectory = false;

        public bool deployChutes = false;
        public int limitChutesStage = 0;

        //simulation inputs:
        public double
            decelEndAltitudeASL =
                0; // The altitude at which we need to have killed velocity - NOTE that this is not the same as the height of the predicted landing site.

        public IDescentSpeedPolicy descentSpeedPolicy = null; //simulate this descent speed policy
        public double parachuteSemiDeployMultiplier = 3; // this will get updated by the autopilot.
        public bool runErrorSimulations = false; // This will be set by the autopilot to turn error simulations on or off.

        //internal data:

        protected bool
            errorSimulationRunning; // the predictor can run two types of simulation - 1) simulations of the current situation. 2) simulations of the current situation with deliberate error introduced into the parachute multiplier to aid the statistical analysis of these results.

        protected readonly Stopwatch errorStopwatch = new Stopwatch();
        protected long millisecondsBetweenErrorSimulations;

        public bool SimulationRunning { get; private set; }

        protected readonly Stopwatch stopwatch = new Stopwatch();
        public double SimulationRunningTime => SimulationRunning ? stopwatch.ElapsedMilliseconds / 1000d : 0;
        protected long millisecondsBetweenSimulations;

        protected ReentrySimulation.Result result;
        protected ReentrySimulation.Result errorResult;
        private ReentrySimulation.Result candidateResult;
        // A landing result is consumed by both the map marker and the landing autopilot.
        // Keep a monotonically increasing version so the autopilot can distinguish a new,
        // accepted prediction from another physics frame using the same result.
        public long ResultVersion { get; private set; }

        // A simulation branch change can move an airless-body impact point by hundreds of
        // metres.  Do not immediately replace a usable result with such an outlier: require
        // the next prediction to corroborate it first.  The tolerance is based on the time
        // between the two input snapshots, rather than several seconds of vessel travel.
        private const double MinimumResultAcceptanceDistance = 35;
        private const double MaximumResultAcceptanceDistance = 200;

        public ManeuverNode aerobrakeNode;

        protected const int interationsPerSecond = 5; // the number of times that we want to try to run the simulation each second.

        protected double
            dt = 0.2; // the suggested dt for each timestep in the simulations. This will be adjusted depending on how long the simulations take to run if variabledt is active

        // TODO - decide if variable for fixed dt results in a more stable result
        protected readonly bool
            variabledt = false; // Set this to true to allow the predictor to choose a dt based on how long each run is taking, and false to use a fixed dt.

        public bool noSkipToFreefall = false;

        private readonly Queue readyResults = new Queue();
        private readonly Queue<TargetAwareWorkCompletion> readyTargetAwareWork =
            new Queue<TargetAwareWorkCompletion>();
        private volatile TargetAwareAirlessPlanner activeTargetAware;
        private Orbit activeTargetAwareOrbit;
        private long activeTargetAwareCaptureId;
        private TargetAwareResultLineage? committedTargetAware;
        private TargetAwareResultLineage? impactLineage;
        private ReentrySimulation.Result impactDiagnostic;
        private ReentrySimulation.Result unavailableTimingView;
        private bool observedDeorbitThrust;
        private ReentrySimulation.LandingForecastKind? lastAvailableForecastKind;
        private bool wasTargetAwareActive;
        private volatile bool predictorDestroyed;
        private long targetAwareSequence;
        private double lastTargetAwareStartUT = double.NegativeInfinity;
        private Type lastTargetAwarePhase;
        private readonly TargetAwareDeliveryLatency targetAwareLatency = new TargetAwareDeliveryLatency();
        private Type activeTargetAwarePhase;
        internal double RecentTargetAwareLatencySeconds => targetAwareLatency.Estimate(false);
        private double targetAwareWorkerMilliseconds;
        private readonly TargetAwareTerrainCache targetAwareTerrainCache = new TargetAwareTerrainCache();
        private readonly TargetAwareTerrainContext approachTerrain = new TargetAwareTerrainContext();
        private readonly TargetAwareBrakingPolicyContext brakingPolicy = new TargetAwareBrakingPolicyContext();
        internal bool TryGetOwnedBrakingTerrain(out double height)
        {
            height = 0;
            return TargetAwareAirlessActive && brakingPolicy.TryGet(Core.Target.targetBody,
                Interlocked.Read(ref predictionGeneration), Core.Target.targetLatitude,
                Core.Target.targetLongitude, Planetarium.GetUniversalTime(), out height);
        }
        private double deliveryReplayBrakeUT = double.NaN;
        private double deliveryReplayTerrainASL;
        private double deliveryReplayWindowEndUT = double.NaN;
        private bool deliveryReplayHasBallisticContact;

        internal bool TryGetValidatedApproachTerrain(out double height)
        {
            height = 0;
            return TargetAwareAirlessActive && approachTerrain.TryGet(Core.Target.targetBody,
                Core.Target.targetLatitude, Core.Target.targetLongitude, out height);
        }
        private long lastMapCaptureVersion = -1;
        private int lastMapCaptureFlags = -1;
        // Cache hits count as samples too. The former 32-sample cap spread
        // inexpensive cached terrain work over seconds of flight time.
        internal const int TargetAwareTerrainSamplesPerTick = 512;
        internal const double TargetAwareRefreshSeconds = 0.25;
        internal const double TargetAwareMaximumSnapshotAgeSeconds = 10;
        // Provisional safety cap. Mun replay measures the actual queries before
        // this becomes an acceptance budget; exceeding it fails the candidate.
        private const int TargetAwareMaximumTerrainQueries = 1536;
        // Each V1 landing target owns a predictor transaction. A simulation
        // can finish after a new target has been selected, so its result must
        // be identified before it reaches the control-facing result queue.
        private long predictionGeneration;

        private Random random;
        public double maxOrbits = 1;

        private double lastSimTime;
        private double lastSimSteps;
        private double lastErrorSimTime;
        private double lastErrorSimSteps;

        [ValueInfoItem("#MechJeb_LandingSim", InfoItem.Category.Misc, showInEditor = false)] //LandingSim
        public string LandingSimTime() =>
            (stopwatch.ElapsedMilliseconds / 1000d).ToString("F1") + "/" + lastSimTime.ToString("F2") + " (" + lastSimSteps + ")\n"
            + (errorStopwatch.ElapsedMilliseconds / 1000d).ToString("F1") + "/" + lastErrorSimTime.ToString("F2") + " (" + lastErrorSimSteps +
            ")\n"
            + ReentrySimulation.ActiveDt.ToString("F2") + " " + ReentrySimulation.ActiveStep + "\n"
            + dt.ToString("F2") + " " + Time.fixedDeltaTime.ToString("F2") + " " + parachuteSemiDeployMultiplier.ToString("F3");

        public override void OnStart(PartModule.StartState state)
        {
            random = new Random();
            if (state != PartModule.StartState.None && state != PartModule.StartState.Editor)
            {
                Core.AddToPostDrawQueue(DoMapView);
            }
        }

        protected override void OnModuleEnabled()
        {
            if (!TargetAwareAirlessActive)
                TryStartSimulation(false);
        }

        private bool TargetAwareAirlessActive => Core.Landing.Enabled && Core.Landing.LandAtTarget &&
            Core.Target.PositionTargetExists && VesselState != null && VesselState.MainBody != null &&
            Core.Target.targetBody == VesselState.MainBody && !VesselState.MainBody.atmosphere;

        protected override void OnModuleDisabled()
        {
            CancelCorrectionTrial(); correctionSourceSnapshot = null;
            ClearRetainedCorrectionForecast();
            ClearImpactDiagnostic();
            observedDeorbitThrust = false;
            targetAwareTerrainCache.Clear();
            approachTerrain.Clear(); brakingPolicy.Clear();
            deliveryReplayBrakeUT = double.NaN;
            targetAwareLatency.Reset();
            wasTargetAwareActive = false;
            Interlocked.Increment(ref predictionGeneration);
            if (Core.Landing.LandingTraceEnabled && Core.Landing.LandAtTarget &&
                VesselState != null && VesselState.MainBody != null && !VesselState.MainBody.atmosphere)
                LandingPredictorCapture.Lifecycle("predictor_disabled", Interlocked.Read(ref predictionGeneration),
                    ResultVersion, result?.CaptureSubmissionId ?? 0);
            CancelTargetAwareWork();
            stopwatch.Stop();
            stopwatch.Reset();

            errorStopwatch.Stop();
            errorStopwatch.Reset();

            if (candidateResult != null)
            {
                LandingPredictorCapture.Discarded(candidateResult.CaptureSubmissionId,
                    "candidate_cleared_on_predictor_disable");
                candidateResult.Release();
                candidateResult = null;
            }
            LandingPredictorCapture.Close();
        }

        public override void OnDestroy()
        {
            if (predictorDestroyed) return;
            predictorDestroyed = true;
            Interlocked.Increment(ref predictionGeneration);
            bool ownedCapture = activeTargetAwareCaptureId != 0 || wasTargetAwareActive;
            CancelTargetAwareWork(false);
            targetAwareTerrainCache.Clear();
            approachTerrain.Clear(); brakingPolicy.Clear();
            deliveryReplayBrakeUT = double.NaN;
            lock (readyResults)
                while (readyResults.Count > 0)
                    ((ReentrySimulation.Result)readyResults.Dequeue()).Release();
            candidateResult?.Release(); candidateResult = null;
            CancelCorrectionTrial();
            ClearRetainedCorrectionForecast();
            result?.Release(); result = null;
            errorResult?.Release(); errorResult = null;
            committedTargetAware = null;
            ClearImpactDiagnostic();
            stopwatch.Stop(); errorStopwatch.Stop();
            if (ownedCapture) LandingPredictorCapture.Close();
            // Workers finish copied mathematics only; never join them on the
            // flight thread or access Core/PQS during scene teardown.
        }

        // A targeted V1 landing is a new predictor transaction. A result or
        // terrain radius from a prior target must never command its first
        // course correction. The landing autopilot calls this before it starts
        // its first phase; no actuator state is changed here.
        public void ResetTargetedLandingPrediction()
        {
            CancelCorrectionTrial(); correctionSourceSnapshot = null;
            ClearRetainedCorrectionForecast();
            ClearImpactDiagnostic();
            observedDeorbitThrust = false;
            lastAvailableForecastKind = null;
            targetAwareTerrainCache.Clear();
            approachTerrain.Clear(); brakingPolicy.Clear();
            deliveryReplayBrakeUT = double.NaN;
            Interlocked.Increment(ref predictionGeneration);
            if (Core.Landing.LandingTraceEnabled && Core.Landing.LandAtTarget &&
                VesselState != null && VesselState.MainBody != null && !VesselState.MainBody.atmosphere)
                LandingPredictorCapture.Lifecycle("target_reset", Interlocked.Read(ref predictionGeneration),
                    ResultVersion, result?.CaptureSubmissionId ?? 0);
            CancelTargetAwareWork();
            committedTargetAware = null;
            lastTargetAwareStartUT = double.NegativeInfinity;
            if (candidateResult != null)
            {
                LandingPredictorCapture.Discarded(candidateResult.CaptureSubmissionId,
                    "candidate_cleared_on_target_reset");
                candidateResult.Release();
                candidateResult = null;
            }
            if (result != null)
            {
                LandingPredictorCapture.Discarded(result.CaptureSubmissionId,
                    "published_cleared_on_target_reset");
                result.Release();
                result = null;
            }
            if (errorResult != null)
            {
                LandingPredictorCapture.Discarded(errorResult.CaptureSubmissionId,
                    "error_result_cleared_on_target_reset");
                errorResult.Release();
                errorResult = null;
            }
        }

        private readonly SustainedBrakingThrust sustainedBraking = new SustainedBrakingThrust();
        private long activeBrakingCapabilityEpoch, publishedBrakingCapabilityEpoch = -1;

        internal double BrakingSpeedPolicyAcceleration()
        {
            double live = VesselState.LimitedMaxThrustAcceleration;
            if (!TargetAwareAirlessActive || !Core.Thrust.DifferentialThrottle ||
                !(Core.Landing.CurrentStep is DecelerationBurn braking) || !braking.BrakingTriggered)
                return live;

            return sustainedBraking.SpeedPolicyAcceleration(VesselState.Time, VesselState.Mass,
                VesselState.ThrustMinimum, Core.Thrust.ThrottleFixedLimit, live);
        }

        private void ObserveBrakingCapability(bool active)
        {
            bool differential = Core.Thrust.DifferentialThrottle;
            // Engine identity, baseline limit and fixed cap are configuration.
            // The allocator's per-tick output is deliberately excluded.
            ulong configuration = differential ? 1UL : 0UL;
            unchecked
            {
                configuration = configuration * 1099511628211UL + (uint)Core.Thrust.ThrottleFixedLimit.GetHashCode();
                foreach (var wrapper in VesselState.EngineWrappers)
                {
                    var engine = wrapper.Engine;
                    var recovery = differential && !engine.throttleLocked ?
                        MechJebDifferentialThrottleState.Find(engine.part) : null;
                    float limit = recovery != null ? recovery.BaselineLimiter(engine) : engine.thrustPercentage;
                    configuration = configuration * 1099511628211UL + (uint)engine.GetInstanceID();
                    configuration = configuration * 1099511628211UL + (uint)limit.GetHashCode();
                }
            }

            bool poweredBraking = active && differential &&
                Core.Landing.CurrentStep is DecelerationBurn braking && braking.BrakingTriggered;
            long previous = sustainedBraking.Epoch;
            sustainedBraking.Observe(VesselState.Time, configuration,
                VesselState.ThrustAvailable, VesselState.MaximumEngineMassFlow, poweredBraking,
                Vessel.ctrlState.mainThrottle > 0);
            if (sustainedBraking.Epoch != previous && active)
            {
                CancelTargetAwareWork();
                CancelCorrectionTrial();
                correctionSourceSnapshot = null;
                lastTargetAwareStartUT = double.NegativeInfinity;
            }
        }

        public override void OnFixedUpdate()
        {
            if (predictorDestroyed) return;
            ObserveTerminalHeading();
            bool targetAware = TargetAwareAirlessActive;
            if (targetAware || wasTargetAwareActive) ObserveBrakingCapability(targetAware);
            if (targetAware && !wasTargetAwareActive)
            {
                observedDeorbitThrust = false;
                // A pre-autoland ordinary result was made by the immediate-burn
                // model and cannot become the first control-facing result.
                if (result != null && !committedTargetAware.HasValue)
                {
                    LandingPredictorCapture.Discarded(result.CaptureSubmissionId,
                        "ordinary_result_invalidated_on_targeted_landing_start");
                    result.Release();
                    result = null;
                    ResultVersion++;
                }
            }
            if (!targetAware && wasTargetAwareActive)
            { CancelCorrectionTrial(); correctionSourceSnapshot = null; ClearRetainedCorrectionForecast(); }
            wasTargetAwareActive = targetAware;
            Type phase = Core.Landing.CurrentStep?.GetType();
            targetAwareLatency.Configure(Interlocked.Read(ref predictionGeneration),
                Core.Target.targetBody, phase, Core.Target.targetLatitude, Core.Target.targetLongitude,
                Planetarium.GetUniversalTime());
            if (targetAware && (Core.Landing.CurrentStep is LowDeorbitBurn ||
                Core.Landing.CurrentStep is DeorbitBurn) && phase == lastTargetAwarePhase &&
                Vessel.ctrlState.mainThrottle > 0 && VesselState.CurrentThrustAcceleration > 0)
                observedDeorbitThrust = true;
            InvalidateChangedTargetAwareContext();
            if (TargetAwareAirlessActive && IsTerminalLandingPhase && phase != lastTargetAwarePhase)
                lastTargetAwareStartUT = double.NegativeInfinity;
            lastTargetAwarePhase = phase;
            if (TargetAwareAirlessActive && IsTerminalLandingPhase &&
                activeTargetAware != null &&
                (!activeTargetAware.IsDirectForecast || activeTargetAware.Snapshot.CoastToDeceleration !=
                    (Core.Landing.CurrentStep is CoastToDeceleration) || activeTargetAware.Snapshot.TerminalPhase !=
                    CurrentTerminalForecastPhase))
                CancelTargetAwareWork();
            CheckForResult();
            CheckForTargetAwareWork();
            bool primaryTerrainOwnedThisTick = activeTargetAware != null;
            AdvanceTargetAwareTransaction();
            AdvanceCorrectionTrial(!primaryTerrainOwnedThisTick);
            if (targetAware)
            {
                ReentrySimulation.LandingForecastKind kind = AvailableForecast()?.ForecastKind ??
                    ReentrySimulation.LandingForecastKind.NoForecast;
                if (kind != lastAvailableForecastKind)
                {
                    lastAvailableForecastKind = kind;
                    LandingPredictorCapture.Decision(impactDiagnostic?.CaptureSubmissionId ??
                        result?.CaptureSubmissionId ?? 0, "target_aware_availability:" + kind,
                        result?.CaptureSubmissionId ?? 0);
                }
            }
            if (TargetAwareAirlessActive)
            {
                if (Core.Landing.CurrentStep is DeorbitBurn ||
                    Core.Landing.CurrentStep is LowDeorbitBurn ||
                    Core.Landing.CurrentStep is CourseCorrection ||
                    Core.Landing.CurrentStep is CoastToDeceleration ||
                    Core.Landing.CurrentStep is DecelerationBurn ||
                    Core.Landing.CurrentStep is KillHorizontalVelocity ||
                    Core.Landing.CurrentStep is FinalDescent)
                    TryStartTargetAwareTransaction();
            }
            else
                TryStartSimulation(true);
        }

        private bool IsTerminalLandingPhase =>
            Core.Landing.CurrentStep is CoastToDeceleration ||
            Core.Landing.CurrentStep is DecelerationBurn ||
            Core.Landing.CurrentStep is KillHorizontalVelocity ||
            Core.Landing.CurrentStep is FinalDescent;

        private int CurrentTerminalForecastPhase => Core.Landing.CurrentStep is FinalDescent ? 2 :
            Core.Landing.CurrentStep is KillHorizontalVelocity ? 1 : 0;

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
        private void ObserveTerminalHeading()
        {
            terminalHeading.Observe(MainBody, VesselState.Forward, VesselState.Time,
                Time.fixedDeltaTime, TimeWarp.CurrentRate == 1);
        }

        private void InvalidateCommittedTargetAware(string reason)
        {
            CancelCorrectionTrial();
            correctionSourceSnapshot = null;
            if (reason != "real_course_correction_pulse_completed")
            { ClearRetainedCorrectionForecast(); approachTerrain.Clear(); brakingPolicy.Clear(); }
            else brakingPolicy.InvalidateForecastAfterPulse();
            deliveryReplayBrakeUT = double.NaN;
            ClearImpactDiagnostic();
            if (!committedTargetAware.HasValue) return;
            if (Core.Landing.LandingTraceEnabled)
                LandingPredictorCapture.Lifecycle(reason, Interlocked.Read(ref predictionGeneration),
                    ResultVersion, result?.CaptureSubmissionId ?? 0);
            if (result != null)
            {
                LandingPredictorCapture.Discarded(result.CaptureSubmissionId, reason);
                if (reason == "real_course_correction_pulse_completed")
                {
                    ClearRetainedCorrectionForecast();
                    retainedCorrectionForecast.Retain(result, result.Body, Core.Target.targetLatitude, Core.Target.targetLongitude);
                }
                else result.Release();
                result = null;
            }
            committedTargetAware = null;
            ResultVersion++;
            lastTargetAwareStartUT = double.NegativeInfinity;
        }

        internal void InvalidateAfterCourseCorrectionPulse()
        {
            if (!TargetAwareAirlessActive) return;
            CancelTargetAwareWork();
            InvalidateCommittedTargetAware("real_course_correction_pulse_completed");
        }

        private void InvalidateChangedTargetAwareContext()
        {
            if (!committedTargetAware.HasValue || !TargetAwareAirlessActive) return;
            TargetAwareResultLineage committed = committedTargetAware.Value;
            if (!ReferenceEquals(committed.Body, Core.Target.targetBody) ||
                committed.TargetLatitude != (double)Core.Target.targetLatitude ||
                committed.TargetLongitude != (double)Core.Target.targetLongitude)
                InvalidateCommittedTargetAware("target_aware_target_changed");
        }

        private void TryStartSimulation(bool doErrorSim)
        {
            try
            {
                if (TargetAwareAirlessActive)
                    return;
                if (!Vessel.LandedOrSplashed)
                {
                    // We should be running simulations periodically. If one is not running right now,
                    // check if enough time has passed since the last one to start a new one:
                    if (!SimulationRunning && (stopwatch.ElapsedMilliseconds > millisecondsBetweenSimulations || !stopwatch.IsRunning))
                    {
                        // variabledt generate too much instability of the landing site with atmo.
                        // variabledt = !(mainBody.atmosphere && core.landing.enabled);
                        // the altitude may induce some instability but allow for greater precision of the display in manual flight

                        //variabledt = !mainBody.atmosphere || vessel.terrainAltitude < 1000 ;
                        //if (!variabledt)
                        //    dt = 0.5;

                        stopwatch.Stop();
                        stopwatch.Reset();

                        StartSimulation(false);
                    }

                    // We also periodically run simulations containing deliberate errors if we have been asked to do so by the landing autopilot.
                    if (doErrorSim && runErrorSimulations && !errorSimulationRunning &&
                        (errorStopwatch.ElapsedMilliseconds >= millisecondsBetweenErrorSimulations || !errorStopwatch.IsRunning))
                    {
                        errorStopwatch.Stop();
                        errorStopwatch.Reset();

                        StartSimulation(true);
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogException(ex);
            }
        }

        public override void OnUpdate() => MaintainAerobrakeNode();

        private LandingPredictorCapture.Snapshot CaptureSnapshot(Orbit orbit, double inputUT, CelestialBody body,
            AirlessTargetAwareSnapshot? activeSnapshot = null,
            double? activeTargetTerrainQueryMs = null)
        {
            if (!Core.Landing.LandingTraceEnabled || !Core.Landing.Enabled || !Core.Landing.LandAtTarget ||
                !Core.Target.PositionTargetExists || Core.Target.targetBody != body || body.atmosphere)
                return null;

            try
            {
                return LandingPredictorCapture.Snapshot.Create(orbit, inputUT, body, VesselState, Core,
                    Core.Target.targetLatitude, Core.Target.targetLongitude,
                    Core.Landing.CurrentStep?.GetType().Name, Interlocked.Read(ref predictionGeneration),
                    activeSnapshot, activeTargetTerrainQueryMs);
            }
            catch (Exception ex)
            {
                // Capture may fail without affecting the existing simulation or control path.
                LandingPredictorCapture.CaptureError("snapshot", ex);
                Debug.Log("[MechJebLandingTrace] passive capture snapshot failed: " + ex.Message);
                return null;
            }
        }

        protected void StartSimulation(bool addParachuteError)
        {
            double altitudeOfPreviousPrediction = 0;
            double parachuteMultiplierForThisSimulation = parachuteSemiDeployMultiplier;
            if (addParachuteError)
            {
                errorSimulationRunning = true;
                errorStopwatch.Start(); //starts a timer that times how long the simulation takes
            }
            else
            {
                SimulationRunning = true;
                stopwatch.Start(); //starts a timer that times how long the simulation takes
            }

            Orbit patch = GetReenteringPatch() ?? Orbit;
            if (patch.referenceBody.atmosphere && result != null && result.Outcome == ReentrySimulation.Outcome.LANDED && result.Body != null)
            {
                altitudeOfPreviousPrediction = result.EndASL;
            }

            // Is this a simulation run with errors added? If so then add some error to the parachute multiple
            if (addParachuteError)
            {
                parachuteMultiplierForThisSimulation *= 1d + (random.Next(1000000) - 500000d) / 10000000d;
            }

            // The curves used for the sim are not thread safe so we need a copy used only by the thread
            var simCurves = ReentrySimulation.SimCurves.Borrow(patch.referenceBody);


            //if (descentSpeedPolicy != null)
            //    print(vesselState.limitedMaxThrustAccel.ToString("F2") + " " + descentSpeedPolicy.MaxAllowedSpeed(vesselState.CoM - mainBody.position, vesselState.surfaceVelocity).ToString("F2"));

            var simVessel = SimulatedVessel.Borrow(Vessel, simCurves, patch.StartUT, Core.Landing.Enabled && deployChutes ? limitChutesStage : -1);
            var sim = ReentrySimulation.Borrow(patch, patch.StartUT, simVessel, simCurves, descentSpeedPolicy, decelEndAltitudeASL,
                VesselState.LimitedMaxThrustAcceleration, parachuteMultiplierForThisSimulation, altitudeOfPreviousPrediction, addParachuteError, dt,
                Time.fixedDeltaTime, maxOrbits, noSkipToFreefall);
            long captureId = 0;
            LandingPredictorCapture.Snapshot capture = CaptureSnapshot(patch, patch.StartUT, patch.referenceBody);
            if (capture != null)
                captureId = LandingPredictorCapture.Submit(capture, addParachuteError ? "parachute_error" : "ordinary",
                    descentSpeedPolicy, decelEndAltitudeASL, altitudeOfPreviousPrediction,
                    VesselState.LimitedMaxThrustAcceleration, parachuteMultiplierForThisSimulation,
                    dt, Time.fixedDeltaTime, maxOrbits, noSkipToFreefall, double.NaN);
            //MechJebCore.print("Sim ran with dt=" + dt.ToString("F3"));

            //Run the simulation in a separate thread
            ThreadPool.QueueUserWorkItem(RunSimulation, new SimulationJob(sim,
                Interlocked.Read(ref predictionGeneration), captureId));
            //RunSimulation(sim);
        }

        private void RunSimulation(object o)
        {
            var job = (SimulationJob)o;
            ReentrySimulation sim = job.Simulation;
            bool workerResultRecorded = false;
            try
            {
                Stopwatch captureWorkerTimer = job.CaptureId != 0 ? Stopwatch.StartNew() : null;
                ReentrySimulation.Result newResult = sim.RunSimulation();
                if (predictorDestroyed)
                {
                    newResult.Release();
                    return;
                }
                captureWorkerTimer?.Stop();
                newResult.CaptureSubmissionId = job.CaptureId;
                LandingPredictorCapture.WorkerResult(job.CaptureId, newResult,
                    job.Generation != Interlocked.Read(ref predictionGeneration),
                    captureWorkerTimer?.Elapsed.TotalMilliseconds ?? 0);
                workerResultRecorded = true;

                // Never let an old target's predictor result or its timing
                // state overwrite the current V1 landing transaction.
                if (job.Generation != Interlocked.Read(ref predictionGeneration))
                {
                    LandingPredictorCapture.Discarded(job.CaptureId, "stale_worker_generation");
                    bool wasErrorSimulation = newResult.MultiplierHasError;
                    newResult.Release();
                    if (wasErrorSimulation)
                    {
                        errorStopwatch.Stop();
                        errorStopwatch.Reset();
                        errorSimulationRunning = false;
                    }
                    else
                    {
                        stopwatch.Stop();
                        stopwatch.Reset();
                        SimulationRunning = false;
                    }
                    return;
                }

                lock (readyResults)
                {
                    if (predictorDestroyed || job.Generation != Interlocked.Read(ref predictionGeneration))
                    {
                        newResult.Release();
                        return;
                    }
                    readyResults.Enqueue(newResult);
                }

                if (newResult.MultiplierHasError)
                {
                    //see how long the simulation took
                    errorStopwatch.Stop();
                    long millisecondsToCompletion = errorStopwatch.ElapsedMilliseconds;
                    lastErrorSimTime = millisecondsToCompletion * 0.001;
                    lastErrorSimSteps = newResult.Steps;

                    errorStopwatch.Reset();

                    // Run error simulations at half the normal rate.  A fast vacuum
                    // simulation used to restart after 5 ms, which made the error
                    // result churn much faster than the physics update.
                    millisecondsBetweenErrorSimulations =
                        Math.Max(2 * 1000 / interationsPerSecond - millisecondsToCompletion, 5);

                    //start the stopwatch that will count off this delay
                    errorStopwatch.Start();
                    errorSimulationRunning = false;
                }
                else
                {
                    //see how long the simulation took
                    stopwatch.Stop();
                    long millisecondsToCompletion = stopwatch.ElapsedMilliseconds;
                    stopwatch.Reset();

                    // Maintain the intended five predictions per second, including
                    // the time spent simulating.  A low-cost vacuum prediction can
                    // complete in a few milliseconds; immediately restarting it
                    // makes the landing marker and its control feedback flicker.
                    millisecondsBetweenSimulations =
                        Math.Max(1000 / interationsPerSecond - millisecondsToCompletion, 5);
                    lastSimTime = millisecondsToCompletion * 0.001;
                    lastSimSteps = newResult.Steps;
                    // Do not wait for too long before running another simulation, but also give the processor a rest.

                    // How long should we set the max_dt to be in the future? Calculate for interationsPerSecond runs per second. If we do not enter the atmosphere, however do not do so as we will complete so quickly, it is not a good guide to how long the reentry simulation takes.
                    if (newResult.Outcome == ReentrySimulation.Outcome.AEROBRAKED ||
                        newResult.Outcome == ReentrySimulation.Outcome.LANDED)
                    {
                        if (variabledt)
                        {
                            dt = newResult.Maxdt * (millisecondsToCompletion / 1000d) / (1d / (3d * interationsPerSecond));
                            // There is no point in having a dt that is smaller than the physics frame rate as we would be trying to be more precise than the game.
                            dt = Math.Max(dt, sim.MinDT);
                            // Set a sensible upper limit to dt as well. - in this case 10 seconds
                            dt = Math.Min(dt, 10);
                        }
                    }

                    //Debug.Log("Result:" + this.result.outcome + " Time to run: " + millisecondsToCompletion + " millisecondsBetweenSimulations: " + millisecondsBetweenSimulations + " new dt: " + dt + " Time.fixedDeltaTime " + Time.fixedDeltaTime + "\n" + this.result.ToString()); // Note the endASL will be zero as it has not yet been calculated, and we are not allowed to calculate it from this thread :(

                    //start the stopwatch that will count off this delay
                    stopwatch.Start();
                    SimulationRunning = false;
                }
            }
            catch (Exception ex)
            {
                //Debug.Log(string.Format("Exception in MechJebModuleLandingPredictions.RunSimulation\n{0}", ex.StackTrace));
                if (!workerResultRecorded)
                    LandingPredictorCapture.WorkerException(job.CaptureId, ex);
                else
                    LandingPredictorCapture.Discarded(job.CaptureId, "post_result_worker_exception");
                //Debug.LogException(ex);
                Dispatcher.InvokeAsync(() => Debug.LogException(ex));
            }
            finally
            {
                sim.Release();
            }
        }

        private sealed class SimulationJob
        {
            public readonly ReentrySimulation Simulation;
            public readonly long Generation;
            public readonly long CaptureId;

            public SimulationJob(ReentrySimulation simulation, long generation, long captureId)
            {
                Simulation = simulation;
                Generation = generation;
                CaptureId = captureId;
            }
        }

        private enum TargetAwareWorkKind { Ballistic, Coarse, Refinement, PolicyEscalation, DirectForecast, Terminal }

        private sealed class TargetAwareWorkItem
        {
            public readonly TargetAwareAirlessPlanner Planner;
            public readonly TargetAwareWorkKind Kind;
            public readonly long CaptureId;

            public TargetAwareWorkItem(TargetAwareAirlessPlanner planner, TargetAwareWorkKind kind,
                long captureId)
            {
                Planner = planner;
                Kind = kind;
                CaptureId = captureId;
            }
        }

        private sealed class TargetAwareWorkCompletion
        {
            public TargetAwareAirlessPlanner Planner;
            public TargetAwareWorkKind Kind;
            public long CaptureId;
            public List<AirlessTargetAwareState> Ballistic;
            public List<AirlessTargetAwareOutput> Coarse;
            public TargetAwareRefinementBatch Refinement;
            public AirlessTargetAwareOutput Revalidated;
            public TargetAwareTerminalEnvelope Terminal;
            public Exception Exception;
            public double ElapsedMilliseconds;
        }

        private void CancelTargetAwareWork(bool recordCapture = true)
        {
            activeTargetAware?.CancelWorkers();
            if (recordCapture && activeTargetAwareCaptureId != 0)
            {
                LandingPredictorCapture.TargetAwareValidation(activeTargetAwareCaptureId,
                    activeTargetAware, targetAwareWorkerMilliseconds);
                LandingPredictorCapture.Abandoned(activeTargetAwareCaptureId,
                    "target_aware_transaction_cancelled");
            }
            activeTargetAware = null;
            activeTargetAwareOrbit = null;
            activeTargetAwareCaptureId = 0;
            lock (readyTargetAwareWork)
                readyTargetAwareWork.Clear();
        }

        private double SafeTouchdownSpeed()
        {
            var ratings = new List<double>(Vessel.parts.Count);
            foreach (Part part in Vessel.parts) ratings.Add(part.crashTolerance);
            return TargetAwareTouchdownSafety.SpeedLimit(ratings, Core.Landing.TouchdownSpeed);
        }

        private void TryStartTargetAwareTransaction()
        {
            if (activeTargetAware != null || Vessel.LandedOrSplashed)
                return;
            bool deliveryReplay = !double.IsNaN(deliveryReplayBrakeUT);
            bool directForecast = IsTerminalLandingPhase || deliveryReplay;
            bool brakingTriggered = Core.Landing.CurrentStep is DecelerationBurn &&
                ((DecelerationBurn)Core.Landing.CurrentStep).BrakingTriggered;
            double fixedCoastReference = directForecast && !brakingTriggered && result != null &&
                result.HasControllerBrakeReferenceUT ? result.ControllerBrakeReferenceUT : double.NaN;
            if (deliveryReplay) fixedCoastReference = deliveryReplayBrakeUT;
            double now = Planetarium.GetUniversalTime();
            if (Core.Landing.CurrentStep is CourseCorrection correction &&
                !correction.PredictionSnapshotSafe(now))
                return;
            // Four submission opportunities per second provide headroom for
            // asynchronous delivery at the requested two publications/second.
            if (now - lastTargetAwareStartUT < TargetAwareRefreshSeconds)
                return;
            lastTargetAwareStartUT = now;
            try
            {
                CelestialBody body = VesselState.MainBody;
                Orbit patch = GetReenteringPatch() ?? Orbit;
                if (patch == null || patch.referenceBody != body || body.pqsController == null)
                    return;
                // The mass, thrust, attitude and policy below are observed now.
                // Use the orbital state at that same UT for one coherent input.
                double inputUT = now;
                Stopwatch targetTerrainTimer = Stopwatch.StartNew();
                double targetTerrainASL = body.TerrainAltitude(Core.Target.targetLatitude,
                    Core.Target.targetLongitude);
                targetTerrainTimer.Stop();
                if (committedTargetAware.HasValue &&
                    committedTargetAware.Value.TargetTerrainASL != targetTerrainASL)
                    InvalidateCommittedTargetAware("target_aware_target_terrain_changed");
                // A pulse invalidates the old endpoint, but correction needs
                // the same nominal braking model revalidated on its new orbit.
                // Retained scalar configuration is a hint, never readiness or
                // permission to republish the pre-pulse trajectory.
                double pulseReference = double.NaN, pulseTerrain = double.NaN;
                bool postPulseSeed = !directForecast && brakingPolicy.TryGetPostPulseSeed(
                    body, Interlocked.Read(ref predictionGeneration), Core.Target.targetLatitude,
                    Core.Target.targetLongitude, targetTerrainASL, now,
                    out pulseReference, out pulseTerrain);
                bool committedSeed = TargetAwarePublicationGate.CanSeedSearch(
                    committedTargetAware, directForecast, Interlocked.Read(ref predictionGeneration), body,
                    Core.Target.targetLatitude, Core.Target.targetLongitude, targetTerrainASL,
                    now, TargetAwareMaximumSnapshotAgeSeconds) &&
                    result != null && result.ForecastKind == ReentrySimulation.LandingForecastKind.LandableForecast &&
                    !double.IsNaN(result.InputForcedBrakingStartUT) &&
                    now >= result.InputUT && now - result.InputUT <= TargetAwareMaximumSnapshotAgeSeconds;
                bool reuseBrakingSeed = committedSeed || postPulseSeed;
                double previousBrakeUT = postPulseSeed ? pulseReference :
                    committedSeed ? result.InputForcedBrakingStartUT : double.NaN;
                // V1 Beta derives its airless handoff height from the published
                // endpoint. Before the first valid result that live value is
                // sea level; a prospective forecast must seed the eventual
                // post-publication policy from resolved terrain, then verify
                // it again against the forecast endpoint before publication.
                double activeDecelEndASL = directForecast ?
                    Core.Landing.DecelerationEndAltitude() :
                    (postPulseSeed ? pulseTerrain :
                     committedSeed ? result.InputProbableLandingSiteASL : targetTerrainASL) + 200;
                if (deliveryReplay) activeDecelEndASL = deliveryReplayTerrainASL + 200;
                bool useSustained = brakingTriggered && Core.Thrust.DifferentialThrottle &&
                    sustainedBraking.HasRecentSample(VesselState.Time);
                double forecastThrust = useSustained ? sustainedBraking.Force : VesselState.ThrustAvailable;
                double forecastFlow = useSustained ? sustainedBraking.Flow : VesselState.MaximumEngineMassFlow;
                double maximumThrustAcceleration = BrakingSpeedPolicyAcceleration();
                bool ongoingDeorbit = (Core.Landing.CurrentStep is LowDeorbitBurn ||
                    Core.Landing.CurrentStep is DeorbitBurn) && Vessel.ctrlState.mainThrottle > 0;
                double deliveryLead = ongoingDeorbit ? Math.Min(2 * TargetAwareRefreshSeconds,
                    targetAwareLatency.Estimate(deliveryReplay, reuseBrakingSeed) + Time.fixedDeltaTime) : 0;
                var activePolicy = new SafeDescentSpeedPolicy(body.Radius + activeDecelEndASL,
                    body.GeeASL * 9.81, maximumThrustAcceleration);
                var rawSnapshot = new AirlessTargetAwareSnapshot(inputUT, now,
                    body.Radius, body.gravParameter, body.GeeASL, body.rotationPeriod,
                    Core.Target.targetLatitude, Core.Target.targetLongitude, targetTerrainASL,
                    activeDecelEndASL, maximumThrustAcceleration, dt,
                    Time.fixedDeltaTime, maxOrbits,
                    body.pqsController.radiusMin - body.Radius,
                    body.pqsController.radiusMax - body.Radius,
                    patch.WorldBCIPositionAtUT(inputUT), patch.WorldOrbitalVelocityAtUT(inputUT),
                    body.angularVelocity, body.GetSurfaceNVector(0, 0),
                    body.GetSurfaceNVector(0, 90), body.GetSurfaceNVector(90, 0), deliveryLead,
                    ongoingDeorbit ? VesselState.Forward * VesselState.CurrentThrustAcceleration : Vector3d.zero,
                    ongoingDeorbit ? VesselState.MinimumEngineMassFlow + Vessel.ctrlState.mainThrottle *
                        (VesselState.MaximumEngineMassFlow - VesselState.MinimumEngineMassFlow) : 0);
                if (!(Core.Landing.DescentSpeedPolicy is SafeDescentSpeedPolicy))
                    throw new InvalidOperationException("V1 airless speed policy unavailable");
                // Read-only projection of the selected Hybrid attitude controller's
                // authority onto the horizontal-kill turn. No live PID is advanced.
                double terminalAlpha = 0, terminalOmega = 0, terminalStoppingTime = 2;
                if (Core.Attitude.Controller is MuMech.AttitudeControllers.HybridController hybrid && hybrid.UseInertia)
                {
                    Vector3d command = V1LandingControlPolicy.HorizontalKillThrustDirection(
                        VesselState.SurfaceVelocity, VesselState.Up);
                    Vector3d worldAxis = Vector3d.Cross(VesselState.Forward, command).normalized;
                    Vector3d axis = Vessel.ReferenceTransform.InverseTransformDirection(worldAxis);
                    for (int i = 0; i < 3; i++)
                        if (VesselState.MoI[i] > 0)
                            terminalAlpha += axis[i] * axis[i] *
                                Math.Max(0, Core.Attitude.torque[i]) / VesselState.MoI[i];
                    terminalOmega = Vector3d.Dot(worldAxis, terminalHeading.Rate);
                    terminalStoppingTime = hybrid.MaxStoppingTime;
                    if (double.IsNaN(terminalAlpha) || double.IsInfinity(terminalAlpha) ||
                        double.IsNaN(terminalOmega) || double.IsInfinity(terminalOmega) ||
                        !(terminalStoppingTime > 0) || double.IsInfinity(terminalStoppingTime))
                    { terminalAlpha = terminalOmega = 0; terminalStoppingTime = 2; }
                }
                var snapshot = new AirlessTargetAwareSnapshot(rawSnapshot,
                    VesselState.Mass, forecastThrust,
                    VesselState.ThrustMinimum, forecastFlow,
                    VesselState.MinimumEngineMassFlow,
                    body.Radius + activeDecelEndASL,
                    activePolicy.Gravity, activePolicy.Thrust, brakingTriggered,
                    VesselState.Forward,
                    Core.Thrust.LimiterMinThrottle ? (double)Core.Thrust.MinThrottle : 0,
                    Math.Max(Core.Thrust.LimiterMinThrottle ?
                        (double)Core.Thrust.MinThrottle : 0,
                        directForecast && !deliveryReplay ? Core.Thrust.ThrottleLimit :
                            Core.Thrust.ThrottleFixedLimit),
                    Core.Thrust.SmoothThrottle ? Core.Thrust.ThrottleSmoothingTime : 0,
                    directForecast && !deliveryReplay ? Core.Thrust.LastThrottle : 0,
                    Core.Landing.TouchdownSpeed,
                    Math.Max(0, VesselState.AltitudeTrue - VesselState.AltitudeBottom),
                    Core.Thrust.PreviousTransThrottle, SafeTouchdownSpeed(),
                    VesselState.TerminalBottomOffset(), VesselState.TerminalControllerBottomOffset(),
                    CurrentTerminalForecastPhase, Core.Thrust.VerticalPidPreviousError,
                    Core.Thrust.VerticalPidIntegral, Core.Landing.CurrentStep is CoastToDeceleration,
                    terminalAlpha, terminalOmega, terminalStoppingTime)
                    .WithEngineCapability(useSustained ? forecastThrust : VesselState.ThrustVectorMaxThrottle.magnitude);
                var orbitCopy = new Orbit();
                orbitCopy.UpdateFromOrbitAtUT(patch, inputUT, body);
                targetAwareTerrainCache.Configure(body, body.pqsController,
                    body.pqsController.radiusMin, body.pqsController.radiusMax, targetTerrainASL);
                var planner = new TargetAwareAirlessPlanner(snapshot,
                    Interlocked.Read(ref predictionGeneration), ++targetAwareSequence, body,
                    (latitude, longitude) => body.TerrainAltitude(latitude, longitude),
                    Math.Max(200, body.Radius * 0.0005), TargetAwareTerrainSamplesPerTick,
                    TargetAwareMaximumTerrainQueries, 1, directForecast, targetAwareTerrainCache,
                    previousBrakeUT, fixedCoastReference, deliveryReplay && !IsTerminalLandingPhase,
                    deliveryReplayWindowEndUT, deliveryReplayHasBallisticContact,
                    CurrentTerminalForecastPhase > 0 && Core.Landing.LandingSurface.Available ?
                        (Func<double, double, double, double>)((latitude, longitude, ground) =>
                        {
                            double surface = Core.Landing.LandingSurface.Altitude(body, latitude, longitude, ground);
                            if (Core.Landing.LandingSurface.LastQuerySaturated)
                                throw new InvalidOperationException("TerminalSurfaceQuerySaturated");
                            return surface;
                        }) : null);
                activeTargetAware = planner;
                activeTargetAwarePhase = Core.Landing.CurrentStep?.GetType();
                deliveryReplayBrakeUT = double.NaN;
                activeTargetAwareOrbit = orbitCopy;
                targetAwareWorkerMilliseconds = 0;
                var activeTargetAwareCapture = CaptureSnapshot(patch, inputUT, body, snapshot,
                    targetTerrainTimer.Elapsed.TotalMilliseconds);
                activeTargetAwareCaptureId = LandingPredictorCapture.Submit(activeTargetAwareCapture,
                    "target_aware_transaction", activePolicy, activeDecelEndASL,
                    targetTerrainASL, snapshot.MaximumThrustAcceleration,
                    parachuteSemiDeployMultiplier, dt, Time.fixedDeltaTime, maxOrbits,
                    noSkipToFreefall, double.NaN,
                    "v1_safe_brake_timing_no_further_correction");
                LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                    "target_aware_refresh_started", result?.CaptureSubmissionId ?? 0);
                lastTargetAwareStartUT = now;
                activeBrakingCapabilityEpoch = sustainedBraking.Epoch;
                QueueTargetAwareWork(planner, directForecast ?
                    TargetAwareWorkKind.DirectForecast : TargetAwareWorkKind.Ballistic);
            }
            catch (Exception ex)
            {
                CancelTargetAwareWork();
                if (Core.Landing.LandingTraceEnabled)
                    Core.Landing.TraceLanding("target-aware submission failed: " + ex.Message);
                Debug.LogException(ex);
            }
        }

        private void QueueTargetAwareWork(TargetAwareAirlessPlanner planner, TargetAwareWorkKind kind)
        {
            if (predictorDestroyed) return;
            if (!ThreadPool.QueueUserWorkItem(RunTargetAwareWork,
                    new TargetAwareWorkItem(planner, kind, activeTargetAwareCaptureId)))
                planner.WorkerFailed("WorkerQueueRejected:" + kind);
        }

        private void RunTargetAwareWork(object state)
        {
            var item = (TargetAwareWorkItem)state;
            if (predictorDestroyed || item.Planner != activeTargetAware ||
                item.Planner.Generation != Interlocked.Read(ref predictionGeneration))
                return;
            var completion = new TargetAwareWorkCompletion
                { Planner = item.Planner, Kind = item.Kind, CaptureId = item.CaptureId };
            Stopwatch timer = Stopwatch.StartNew();
            try
            {
                switch (item.Kind)
                {
                    case TargetAwareWorkKind.Ballistic:
                        completion.Ballistic = AirlessTargetAwareSimulation.BallisticTerrainPass(
                            item.Planner.Snapshot);
                        break;
                    case TargetAwareWorkKind.Coarse:
                        completion.Coarse = item.Planner.RunCoarse();
                        break;
                    case TargetAwareWorkKind.Refinement:
                        completion.Refinement = item.Planner.RunRefinement();
                        break;
                    case TargetAwareWorkKind.PolicyEscalation:
                        completion.Revalidated = item.Planner.RunPolicyEscalation();
                        break;
                    case TargetAwareWorkKind.DirectForecast:
                        // Revalidate the actual post-cutoff orbit throughout
                        // coast. This is one fixed-reference forecast, not a
                        // candidate search or an ordinary predictor fallback.
                        completion.Revalidated = item.Planner.RunDirectForecast();
                        break;
                    case TargetAwareWorkKind.Terminal:
                        completion.Terminal = item.Planner.RunTerminalEnvelope();
                        break;
                }
            }
            catch (Exception ex)
            {
                completion.Exception = ex;
            }
            finally
            {
                timer.Stop();
                completion.ElapsedMilliseconds = timer.Elapsed.TotalMilliseconds;
                lock (readyTargetAwareWork)
                    if (!predictorDestroyed && item.Planner == activeTargetAware &&
                        item.Planner.Generation == Interlocked.Read(ref predictionGeneration))
                        readyTargetAwareWork.Enqueue(completion);
            }
        }

        private void CheckForTargetAwareWork()
        {
            lock (readyTargetAwareWork)
            {
                while (readyTargetAwareWork.Count > 0)
                {
                    TargetAwareWorkCompletion work = readyTargetAwareWork.Dequeue();
                    int outputCount = work.Kind == TargetAwareWorkKind.PolicyEscalation ||
                                      work.Kind == TargetAwareWorkKind.DirectForecast ?
                        (work.Revalidated == null ? 0 : 1) :
                        work.Kind == TargetAwareWorkKind.Terminal ?
                        (work.Terminal == null ? 0 : 2) :
                        work.Kind == TargetAwareWorkKind.Ballistic ?
                        work.Ballistic?.Count ?? 0 : work.Kind == TargetAwareWorkKind.Coarse ?
                            work.Coarse?.Count ?? 0 : work.Refinement?.Outputs?.Count ?? 0;
                    LandingPredictorCapture.TargetAwareWorkerStage(work.CaptureId,
                        work.Kind.ToString(), work.ElapsedMilliseconds, outputCount,
                        work.Exception?.GetType().Name);
                    if (work.Planner != activeTargetAware ||
                        work.Planner.Generation != Interlocked.Read(ref predictionGeneration))
                        continue;
                    targetAwareWorkerMilliseconds += work.ElapsedMilliseconds;
                    if (work.Exception != null)
                    {
                        work.Planner.WorkerFailed("Worker" + work.Kind + ":" +
                            (work.Exception is ArgumentException && work.Planner.Snapshot.MaximumThrustAcceleration <= 0 ?
                                "NonPositiveForwardThrust" : work.Exception.GetType().Name));
                        Debug.LogException(work.Exception);
                        continue;
                    }
                    try
                    {
                        switch (work.Kind)
                        {
                            case TargetAwareWorkKind.Ballistic:
                                work.Planner.SetBallisticSamples(work.Ballistic);
                                break;
                            case TargetAwareWorkKind.Coarse:
                                work.Planner.SetCoarseOutputs(work.Coarse);
                                break;
                            case TargetAwareWorkKind.Refinement:
                                work.Planner.SetRefinement(work.Refinement);
                                break;
                            case TargetAwareWorkKind.PolicyEscalation:
                                work.Planner.SetPolicyEscalation(work.Revalidated);
                                break;
                            case TargetAwareWorkKind.DirectForecast:
                                work.Planner.SetDirectOutput(work.Revalidated);
                                break;
                            case TargetAwareWorkKind.Terminal:
                                work.Planner.SetTerminalEnvelope(work.Terminal);
                                break;
                        }
                    }
                    catch (Exception ex)
                    {
                        work.Planner.WorkerFailed("IncompleteWorkerCompletion:" +
                            work.Kind + ":" + ex.GetType().Name);
                        Debug.LogException(ex);
                    }
                }
            }
        }

        // Exercise this same flight-thread dispatcher in offline cadence tests.
        internal static void AdvanceTargetAwareTerrain(TargetAwareAirlessPlanner planner)
        {
            if (planner.Stage == TargetAwarePlannerStage.ResolveBallistic ||
                planner.Stage == TargetAwarePlannerStage.ResolveCoarse ||
                planner.Stage == TargetAwarePlannerStage.ResolveRefinement ||
                planner.Stage == TargetAwarePlannerStage.ResolvePolicyEscalation ||
                planner.Stage == TargetAwarePlannerStage.ResolveDirectForecast ||
                planner.Stage == TargetAwarePlannerStage.ResolveSelectedCoast ||
                planner.Stage == TargetAwarePlannerStage.ResolveTerminalNominal ||
                planner.Stage == TargetAwarePlannerStage.ResolveTerminalDelayed ||
                planner.Stage == TargetAwarePlannerStage.ResolveBrakingResponse)
                planner.AdvanceTerrain();
        }

        private void AdvanceTargetAwareTransaction()
        {
            TargetAwareAirlessPlanner planner = activeTargetAware;
            if (planner == null)
                return;
            if (!TargetAwareAirlessActive ||
                planner.Generation != Interlocked.Read(ref predictionGeneration) ||
                !ReferenceEquals(planner.BodyIdentity, Core.Target.targetBody) ||
                planner.Snapshot.TargetLatitude != (double)Core.Target.targetLatitude ||
                planner.Snapshot.TargetLongitude != (double)Core.Target.targetLongitude)
            {
                CancelTargetAwareWork();
                return;
            }
            double age = Planetarium.GetUniversalTime() - planner.Snapshot.EpochUT;
            if (age < 0 || age > TargetAwareMaximumSnapshotAgeSeconds)
                planner.WorkerFailed("SnapshotExpiredBeforePublication");
            else
                planner.RefreshRejectedMovingSnapshot(age,
                    (Core.Landing.CurrentStep is DeorbitBurn ||
                     Core.Landing.CurrentStep is LowDeorbitBurn) &&
                    Vessel.ctrlState.mainThrottle > 0 && committedTargetAware.HasValue,
                    TargetAwareRefreshSeconds);
            AdvanceTargetAwareTerrain(planner);

            if (planner.Stage == TargetAwarePlannerStage.ReadyCoarse)
            {
                planner.BeginCoarseWorker();
                QueueTargetAwareWork(planner, TargetAwareWorkKind.Coarse);
            }
            else if (planner.Stage == TargetAwarePlannerStage.ReadyRefinement)
            {
                planner.BeginRefinementWorker();
                QueueTargetAwareWork(planner, TargetAwareWorkKind.Refinement);
            }
            else if (planner.Stage == TargetAwarePlannerStage.ReadyPolicyEscalation)
            {
                planner.BeginPolicyEscalation();
                QueueTargetAwareWork(planner, TargetAwareWorkKind.PolicyEscalation);
            }
            else if (planner.Stage == TargetAwarePlannerStage.ReadyTerminal)
            {
                planner.BeginTerminalWorker();
                QueueTargetAwareWork(planner, TargetAwareWorkKind.Terminal);
            }
            else if (planner.Stage == TargetAwarePlannerStage.Complete)
            {
                // Learn before acceptance: impacts and completed delivery
                // rechecks are measurements too. Cancellations never get here.
                targetAwareLatency.ObserveCompleted(planner.Generation, planner.BodyIdentity,
                    activeTargetAwarePhase, planner.Snapshot.TargetLatitude, planner.Snapshot.TargetLongitude,
                    planner.Snapshot.InputUT, Planetarium.GetUniversalTime(), planner.IsDirectForecast,
                    planner.UsedLocalSearch && !planner.BroadSearchFallback);
                bool powered = (Core.Landing.CurrentStep is LowDeorbitBurn ||
                    Core.Landing.CurrentStep is DeorbitBurn) && Vessel.ctrlState.mainThrottle > 0;
                if (planner.SelectedOutput != null && !double.IsNaN(planner.SelectedOutput.BrakeUT) &&
                    powered &&
                    !TargetAwarePublicationGate.DeliveryResidualWithinBudget(planner.EvaluatedSnapshot,
                        Planetarium.GetUniversalTime(), Orbit.WorldBCIPositionAtUT(Planetarium.GetUniversalTime()),
                        Orbit.WorldOrbitalVelocityAtUT(Planetarium.GetUniversalTime()), VesselState.Mass,
                        planner.SelectedOutput.BrakeUT, Math.Max(200, VesselState.MainBody.Radius * .0005), out _))
                {
                    // Revalidate from a NEW physical snapshot after a slow
                    // terrain search, never relabel the old input epoch.
                    deliveryReplayBrakeUT = planner.SelectedOutput.BrakeUT;
                    deliveryReplayTerrainASL = planner.SelectedPolicyTerrainASL;
                    deliveryReplayWindowEndUT = planner.BrakeSearchWindowEndUT;
                    deliveryReplayHasBallisticContact = planner.BallisticContactFound || planner.InheritedBallisticContactWindow;
                    LandingPredictorCapture.TargetAwareValidation(activeTargetAwareCaptureId,
                        planner, targetAwareWorkerMilliseconds);
                    LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                        "target_aware_delivery_replay_required", result?.CaptureSubmissionId ?? 0);
                    LandingPredictorCapture.Abandoned(activeTargetAwareCaptureId, "delivery_replay_required");
                    lastTargetAwareStartUT = double.NegativeInfinity;
                }
                else TryPublishTargetAwareResult(planner);
                activeTargetAware = null;
                activeTargetAwareOrbit = null;
                activeTargetAwareCaptureId = 0;
            }
            else if (planner.Stage == TargetAwarePlannerStage.Uncertain)
            {
                LandingPredictorCapture.TargetAwareValidation(activeTargetAwareCaptureId,
                    planner, targetAwareWorkerMilliseconds);
                LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                    "target_aware_uncertain:attitude_gate_held", result?.CaptureSubmissionId ?? 0);
                LandingPredictorCapture.Abandoned(activeTargetAwareCaptureId, "attitude_gate_held");
                CancelTargetAwareWork(false);
            }
            else if (planner.Stage == TargetAwarePlannerStage.Failed)
            {
                if (planner.Failure != "SnapshotExpiredBeforePublication" &&
                    planner.Failure != "MovingDeorbitSnapshotRefresh" &&
                    !planner.Failure.StartsWith("Worker", StringComparison.Ordinal))
                    targetAwareLatency.ObserveCompleted(planner.Generation, planner.BodyIdentity,
                        activeTargetAwarePhase, planner.Snapshot.TargetLatitude, planner.Snapshot.TargetLongitude,
                        planner.Snapshot.InputUT, Planetarium.GetUniversalTime(), planner.IsDirectForecast,
                        planner.UsedLocalSearch && !planner.BroadSearchFallback);
                if (Core.Landing.LandingTraceEnabled)
                    Core.Landing.TraceLanding("target-aware transaction failed: " + planner.Failure +
                        " terrainQueries=" + planner.TerrainQueryCount);
                LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                    "target_aware_failed:" + planner.Failure, result?.CaptureSubmissionId ?? 0);
                CancelTargetAwareWork();
            }
        }

        private void TryPublishTargetAwareResult(TargetAwareAirlessPlanner planner)
        {
            ReentrySimulation.Result replacement = null;
            try
            {
                LandingPredictorCapture.TargetAwareValidation(activeTargetAwareCaptureId,
                    planner, targetAwareWorkerMilliseconds);
                if (activeBrakingCapabilityEpoch != sustainedBraking.Epoch)
                    throw new InvalidOperationException("Engine capability changed before publication");
                var body = (CelestialBody)planner.BodyIdentity;
                AirlessTargetAwareSnapshot snapshot = planner.Snapshot;
                if (!snapshot.HasV1ControlModel || planner.SelectedOutput == null ||
                    !planner.SelectedOutput.UsesV1ControlModel)
                    throw new InvalidOperationException("Non-V1 model cannot publish");
                bool impactForecast = planner.SelectedTerrain != null &&
                    (planner.SelectedTerrain.HasFirstContact ||
                     planner.TerminalImpactForecast);
                bool landableForecast = planner.TerminalTouchdownValidated;
                if (!planner.TerminalSurfaceStillValid())
                    throw new InvalidOperationException("TerminalSurfaceChangedBeforePublication");
                AirlessTargetAwareState contact = planner.NominalTerminalTerrain.HasContact ?
                    planner.NominalTerminalTerrain.Contact : planner.SelectedTerrain.FirstContact;
                double contactTerrain = planner.NominalTerminalTerrain.HasContact ?
                    planner.NominalTerminalTerrain.ContactTerrainASL : planner.SelectedTerrain.LocalTerrainASL;
                if (!TargetAwareTerrainContext.ContactValid(snapshot, contact, contactTerrain))
                    throw new InvalidOperationException("Invalid forecast contact geometry");
                // A braking handoff remains an intermediate state. Only a
                // terrain-resolved terminal continuation may publish LANDED.
                if (!impactForecast && !landableForecast)
                {
                    LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                        "baseline_terminal_continuation_unresolved",
                        result?.CaptureSubmissionId ?? 0);
                    LandingPredictorCapture.Abandoned(activeTargetAwareCaptureId,
                        "baseline_terminal_continuation_unresolved");
                    return;
                }
                if (Core.Landing.CurrentStep is CourseCorrection correction &&
                    !correction.PredictionSnapshotSafe(snapshot.InputUT))
                {
                    LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                        "target_aware_rejected:correction_pulse_changed_orbit",
                        result?.CaptureSubmissionId ?? 0);
                    LandingPredictorCapture.Abandoned(activeTargetAwareCaptureId,
                        "correction_pulse_changed_orbit");
                    lastTargetAwareStartUT = double.NegativeInfinity;
                    return;
                }
                double currentTerrain = body.TerrainAltitude(Core.Target.targetLatitude,
                    Core.Target.targetLongitude);
                if (committedTargetAware.HasValue &&
                    committedTargetAware.Value.TargetTerrainASL != currentTerrain)
                    InvalidateCommittedTargetAware("target_aware_target_terrain_changed");
                var lineage = new TargetAwareResultLineage(planner.Generation, planner.Sequence,
                    body, snapshot.TargetLatitude, snapshot.TargetLongitude,
                    snapshot.TargetTerrainASL, snapshot.EpochUT, snapshot.InputUT,
                    true, impactForecast || landableForecast,
                    planner.SelectedTerrain != null && planner.SelectedTerrain.Resolved &&
                        (planner.SelectedTerrain.HasFirstContact ||
                         (planner.NominalTerminalTerrain.Resolved &&
                          planner.DelayedTerminalTerrain.Resolved)),
                    planner.SelectedTerrain != null &&
                        (planner.SelectedTerrain.HasFirstContact ||
                         planner.SelectedTerrain.ClearPath),
                    landableForecast && planner.TerminalHandoff.NecessaryControlBoundPasses,
                    landableForecast ? ReentrySimulation.LandingForecastKind.LandableForecast :
                        ReentrySimulation.LandingForecastKind.ImpactForecast,
                    impactForecast, landableForecast);
                TargetAwarePublicationDecision decision = TargetAwarePublicationGate.CheckObservation(lineage,
                    impactLineage ?? committedTargetAware, Interlocked.Read(ref predictionGeneration),
                    Core.Target.targetBody, Core.Target.targetLatitude, Core.Target.targetLongitude,
                    currentTerrain, Planetarium.GetUniversalTime(),
                    TargetAwareMaximumSnapshotAgeSeconds, 0);
                if (decision != TargetAwarePublicationDecision.Accept)
                {
                    LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                        "target_aware_rejected:" + decision, result?.CaptureSubmissionId ?? 0);
                    LandingPredictorCapture.Abandoned(activeTargetAwareCaptureId,
                        "target_aware_publication_rejected:" + decision);
                    if (Core.Landing.LandingTraceEnabled)
                        Core.Landing.TraceLanding("target-aware publication rejected: " + decision);
                    return;
                }

                if (Core.Landing.CurrentStep is DecelerationBurn terrainBraking &&
                    !double.IsNegativeInfinity(terrainBraking.RaisedHandoffASL) &&
                    snapshot.DecelEndASL < terrainBraking.RaisedHandoffASL)
                {
                    LandingPredictorCapture.Abandoned(activeTargetAwareCaptureId, "braking_terrain_floor_changed");
                    lastTargetAwareStartUT = double.NegativeInfinity;
                    return;
                }
                replacement = BuildTargetAwareResult(planner, body);
                if (landableForecast)
                {
                    bool retained = IsTerminalLandingPhase && TryGetOwnedBrakingTerrain(out _);
                    replacement.OwnedBrakingPlanId = retained ? brakingPolicy.PlanId : planner.Sequence;
                    if (retained)
                    {
                        replacement.ControllerBrakeReferenceUT = brakingPolicy.ReferenceUT;
                        replacement.ControllerBrakeReferencePosition = result.ControllerBrakeReferencePosition;
                    }
                }
                approachTerrain.Observe(body, snapshot.TargetLatitude, snapshot.TargetLongitude,
                    replacement.HasSeparateTerminalSurface ?
                        body.TerrainAltitude(replacement.EndPosition.Latitude, replacement.EndPosition.Longitude) :
                        replacement.EndASL, planner.Sequence);
                LandingPredictorCapture.WorkerResult(activeTargetAwareCaptureId, replacement,
                    false, targetAwareWorkerMilliseconds);
                LandingPredictorCapture.ResolvedResult(activeTargetAwareCaptureId, replacement,
                    null, "target_aware_clearance_validated", planner.Generation,
                    Core.Landing.CurrentStep?.GetType().Name);
                if (impactForecast)
                {
                    publishedBrakingCapabilityEpoch = activeBrakingCapabilityEpoch;
                    LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                        "target_aware_fresh_impact_contradicts_older_landing",
                        result?.CaptureSubmissionId ?? 0);
                    if (impactDiagnostic != null)
                    {
                        LandingPredictorCapture.Discarded(impactDiagnostic.CaptureSubmissionId,
                            "impact_diagnostic_replaced");
                        impactDiagnostic.Release();
                    }
                    ClearRetainedCorrectionForecast();
                    CancelCorrectionTrial(); correctionSourceSnapshot = null;
                    impactDiagnostic = replacement;
                    replacement = null;
                    impactLineage = lineage;
                    unavailableTimingView = TargetAwareForecastPresentation.UnavailableTimingView(
                        result ?? impactDiagnostic);
                    ResultVersion++;
                    LandingPredictorCapture.Published(impactDiagnostic.CaptureSubmissionId,
                        ResultVersion, planner.Generation, Core.Landing.CurrentStep?.GetType().Name);
                    return;
                }
                LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                    "target_aware_atomic_accept", result?.CaptureSubmissionId ?? 0);
                bool raisedHandoff = Core.Landing.CurrentStep is DecelerationBurn braking &&
                    !double.IsNegativeInfinity(braking.RaisedHandoffASL);
                double commitTerrain = raisedHandoff && TryGetOwnedBrakingTerrain(out double ownedTerrain)
                    ? ownedTerrain : planner.SelectedPolicyTerrainASL;
                if (!brakingPolicy.Commit(body, planner.Generation, snapshot.TargetLatitude,
                    snapshot.TargetLongitude, planner.Sequence, snapshot.InputUT,
                    commitTerrain, replacement.ControllerBrakeReferenceUT,
                    IsTerminalLandingPhase, snapshot.TargetTerrainASL))
                    throw new InvalidOperationException("Incompatible braking policy publication");
                ReentrySimulation.Result predecessor = result;
                ClearRetainedCorrectionForecast();
                result = replacement;
                correctionSourceSnapshot = planner.Snapshot.WithV1LandingTerrain(planner.SelectedPolicyTerrainASL);
                replacement = null;
                ClearImpactDiagnostic();
                unavailableTimingView = TargetAwareForecastPresentation.UnavailableTimingView(result);
                committedTargetAware = lineage;
                publishedBrakingCapabilityEpoch = activeBrakingCapabilityEpoch;
                ResultVersion++;
                LandingPredictorCapture.Published(result.CaptureSubmissionId, ResultVersion,
                    planner.Generation, Core.Landing.CurrentStep?.GetType().Name);
                if (predecessor != null)
                {
                    LandingPredictorCapture.Discarded(predecessor.CaptureSubmissionId,
                        "published_result_replaced_by_target_aware");
                    predecessor.Release();
                }
                if (Core.Landing.LandingTraceEnabled)
                    Core.Landing.TraceLanding($"target-aware committed seq={planner.Sequence} " +
                        $"inputUT={snapshot.InputUT:F2} brakeUT={planner.SelectedOutput.BrakeUT:F2} " +
                        $"downrange={planner.SignedDownrangeError:F1} crossrange={planner.CrossrangeError:F1} " +
                        $"clearance={planner.SelectedTerrain.HandoffClearance:F1} " +
                        $"vertical={result.EndVerticalSpeed:F1} " +
                        $"terrainQueries={planner.TerrainQueryCount} " +
                        $"terrainMs={planner.TerrainQueryMilliseconds:F3} workerMs={targetAwareWorkerMilliseconds:F3}");
            }
            catch (Exception ex)
            {
                if (replacement != null)
                    replacement.Release();
                LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                    "target_aware_publication_exception:" + ex.GetType().Name,
                    result?.CaptureSubmissionId ?? 0);
                LandingPredictorCapture.Abandoned(activeTargetAwareCaptureId,
                    "target_aware_publication_exception");
                Debug.LogException(ex);
            }
        }

        private ReentrySimulation.Result BuildTargetAwareResult(TargetAwareAirlessPlanner planner,
            CelestialBody body)
        {
            AirlessTargetAwareSnapshot snapshot = planner.Snapshot;
            AirlessTargetAwareOutput output = planner.SelectedOutput;
            ReentrySimulation.Result published = ReentrySimulation.Result.Borrow();
            published.Trajectory = null;
            try
            {
                bool terminalResult = planner.TerminalEnvelope != null &&
                    planner.NominalTerminalTerrain.HasContact;
                bool landableResult = terminalResult && planner.TerminalTouchdownValidated;
                if (!planner.SelectedTerrain.HasFirstContact && !terminalResult)
                    throw new InvalidOperationException("Terminal path is unresolved");
                published.Outcome = landableResult ? ReentrySimulation.Outcome.LANDED :
                    ReentrySimulation.Outcome.IMPACT;
                published.ForecastKind = landableResult ?
                    ReentrySimulation.LandingForecastKind.LandableForecast :
                    ReentrySimulation.LandingForecastKind.ImpactForecast;
                published.Exception = null;
                published.Body = body;
                published.ReferenceFrame = new ReferenceFrame();
                published.ReferenceFrame.UpdateAtCurrentTime(body);
                published.ID = unchecked((ulong)planner.Sequence);
                published.InputInitialOrbit = activeTargetAwareOrbit;
                published.InputUT = snapshot.InputUT;
                if (planner.BrakingResponse?.Validated == true &&
                    (landableResult || planner.BrakingResponse.ImpactBaseline))
                {
                    var response=planner.BrakingResponse;
                    published.PoweredBrakingResponse=new PublishedBrakingResponse(
                        AirlessTargetAwareSimulation.ToAbsolute(response.Directions[0],snapshot.EpochUT,snapshot),
                        AirlessTargetAwareSimulation.ToAbsolute(response.Directions[1],snapshot.EpochUT,snapshot),
                        AirlessTargetAwareSimulation.ToAbsolute(response.RadialResponse,snapshot.EpochUT,snapshot),
                        AirlessTargetAwareSimulation.ToAbsolute(response.NormalResponse,snapshot.EpochUT,snapshot),
                        AirlessTargetAwareSimulation.ToAbsolute(response.SteeringCommand,snapshot.EpochUT,snapshot),
                        response.Horizon,response.MaximumRatio);
                }
                published.InputDescentSpeedPolicy = new SafeDescentSpeedPolicy(
                    body.Radius + planner.SelectedPolicyTerrainASL + 200,
                    snapshot.BodyGeeASL * 9.81, snapshot.MaximumThrustAcceleration);
                published.InputDecelEndAltitudeASL = planner.SelectedPolicyTerrainASL + 200;
                published.InputMaxThrustAccel = snapshot.MaximumThrustAcceleration;
                published.InputProbableLandingSiteASL = planner.SelectedPolicyTerrainASL;
                published.InputParachuteSemiDeployMultiplier = 0;
                published.InputMultiplierHasError = false;
                published.InputDT = snapshot.Dt;
                published.InputMaxOrbits = snapshot.MaxOrbits;
                published.InputNoSkipToFreefall = false;
                AirlessTargetAwareState endpoint = terminalResult ?
                    planner.NominalTerminalTerrain.Contact :
                    planner.SelectedTerrain.FirstContact;
                bool burnBeginsBeforeContact = !double.IsNaN(output.BrakeUT) &&
                    output.Trajectory.Count > 0 &&
                    output.Trajectory[0].UT <= endpoint.UT && output.BrakeUT <= output.End.UT;
                published.InputForcedBrakingStartUT = burnBeginsBeforeContact ?
                    output.BrakeUT : double.NaN;
                published.InputParachuteList = null;
                published.MultiplierHasError = false;
                published.ParachuteMultiplier = 0;
                published.AeroBrake = false;
                published.AeroBrakeUT = 0;
                published.AeroBrakePosition = default(AbsoluteVector);
                published.AeroBrakeVelocity = default(AbsoluteVector);
                published.DebugLog = null;
                published.SimulatedBrakingStartUT = burnBeginsBeforeContact ?
                    output.Trajectory[0].UT : double.NaN;
                published.HasControllerBrakeReferenceUT = burnBeginsBeforeContact;
                published.ControllerBrakeReferenceUT = burnBeginsBeforeContact ?
                    output.BrakeUT : double.NaN;
                published.HasFirstRequiredThrust = landableResult && snapshot.TerminalPhase == 0 &&
                    output.FirstRequiredThrust.HasValue && !output.AttitudeUncertain &&
                    output.FirstRequiredThrust.Value.UT >= snapshot.StateUT &&
                    output.FirstRequiredThrust.Value.UT < endpoint.UT;
                if (published.HasFirstRequiredThrust)
                {
                    AirlessTargetAwareState first = output.FirstRequiredThrust.Value;
                    published.FirstRequiredThrustUT = first.UT;
                    published.FirstRequiredThrustPosition = AirlessTargetAwareSimulation.ToAbsolute(
                        first.Position, first.UT, snapshot);
                    published.FirstRequiredThrustVelocity = AirlessTargetAwareSimulation.ToAbsolute(
                        first.Velocity, first.UT, snapshot);
                }
                if (burnBeginsBeforeContact)
                {
                    // RunV1Control releases at BrakeUT - 5. The live consumer
                    // must use that same reference, independently of the first
                    // propagated path state (which may already be five seconds
                    // earlier). The displayed path still starts at its true UT.
                    AirlessTargetAwareState reference = output.BrakeReferenceState();
                    published.ControllerBrakeReferencePosition =
                        AirlessTargetAwareSimulation.ToAbsolute(reference.Position,
                            reference.UT, snapshot);
                }
                if (snapshot.TerminalPhase != 0)
                {
                    // Terminal re-anchoring does not invent a new braking
                    // location in the past. Preserve the executed reference.
                    published.HasControllerBrakeReferenceUT = result != null && result.HasControllerBrakeReferenceUT;
                    published.ControllerBrakeReferenceUT = published.HasControllerBrakeReferenceUT ?
                        result.ControllerBrakeReferenceUT : double.NaN;
                    published.ControllerBrakeReferencePosition = published.HasControllerBrakeReferenceUT ?
                        result.ControllerBrakeReferencePosition : default(AbsoluteVector);
                    published.SimulatedBrakingStartUT = result?.SimulatedBrakingStartUT ?? double.NaN;
                }
                published.StartPosition = AirlessTargetAwareSimulation.ToAbsolute(
                    snapshot.Position, snapshot.InputUT, snapshot);
                published.EndPosition = AirlessTargetAwareSimulation.ToAbsolute(
                    endpoint.Position, endpoint.UT, snapshot);
                published.EndVelocity = AirlessTargetAwareSimulation.ToAbsolute(
                    endpoint.Velocity, endpoint.UT, snapshot);
                published.EndUT = endpoint.UT;
                published.EndASL = terminalResult ?
                    planner.NominalTerminalTerrain.ContactTerrainASL :
                    planner.SelectedTerrain.LocalTerrainASL;
                published.HasSeparateTerminalSurface = planner.TerminalSurfaceSamples.Count > 0;
                published.TimeToComplete = endpoint.UT - snapshot.InputUT;
                published.EndSurfaceSpeed = (endpoint.Velocity -
                    Vector3d.Cross(snapshot.AngularVelocity, endpoint.Position)).magnitude;
                Vector3d endUp = endpoint.Position.normalized;
                Vector3d endSurface = endpoint.Velocity -
                    Vector3d.Cross(snapshot.AngularVelocity, endpoint.Position);
                published.EndVerticalSpeed = Vector3d.Dot(endSurface, endUp);
                published.EndHorizontalSpeed = Vector3d.Exclude(endUp, endSurface).magnitude;
                published.EndTerrainClearance = endpoint.Position.magnitude -
                    body.Radius - published.EndASL -
                    (terminalResult ? snapshot.TerminalBottomOffset : snapshot.BottomOffset);
                // The optional braking response can exceed the legacy 0.1
                // ratio, bounded by the existing 0.75 alignment gate. This
                // bounds only the excluded direct steering component.
                double poweredDuration = burnBeginsBeforeContact ?
                    Math.Max(0, Math.Min(endpoint.UT, output.End.UT) -
                        output.Trajectory[0].UT) : 0;
                published.ExcludedTargetSteeringDisplacementBound =
                    Math.Max(0, output.VirtualDeltaV) * poweredDuration *
                    Math.Sqrt(1-.75*.75);
                // The worker may have continued past first physical contact.
                // Its full-run delta-v is not the impact trajectory's delta-v.
                published.DeltaVExpended = terminalResult ?
                    (snapshot.TerminalPhase == 0 ? output.VirtualDeltaV : 0) +
                    LandingForecastInformation.ImpulseAt(planner.TerminalEnvelope.Nominal.Trajectory, endpoint.UT) :
                    LandingForecastInformation.ImpulseAt(output.Trajectory, endpoint.UT);
                published.DeltaVExpended += LandingForecastInformation.DeliveryImpulse(snapshot.ObservedThrustAcceleration.magnitude,
                    snapshot.InitialMass, snapshot.ObservedMassFlow, snapshot.DeliveryLeadSeconds);
                published.MaxDragGees = 0;
                published.Maxdt = snapshot.Dt;
                published.Steps = output.Steps;
                published.Prediction = default(ReentrySimulation.Prediction);
                published.CaptureSubmissionId = activeTargetAwareCaptureId;
                // Display the propagated path at its real UT. DecelerationBurn
                // uses the explicit controller reference for its coast guard.
                // The unpowered coast is represented by the current orbit,
                // never prepended to the powered trajectory.
                published.Trajectory = new List<AbsoluteVector>(
                    output.Trajectory.Count + 1);
                foreach (AirlessTargetAwareState state in output.Trajectory)
                {
                    if (state.UT > endpoint.UT) break;
                    published.Trajectory.Add(AirlessTargetAwareSimulation.ToAbsolute(
                        state.Position, state.UT, snapshot));
                }
                if (terminalResult)
                    foreach (AirlessTargetAwareState state in planner.TerminalEnvelope.Nominal.Trajectory)
                    {
                        if (state.UT > endpoint.UT) break;
                        published.Trajectory.Add(AirlessTargetAwareSimulation.ToAbsolute(
                            state.Position, state.UT, snapshot));
                    }
                return published;
            }
            catch
            {
                published.Release();
                throw;
            }
        }

        private void CheckForResult()
        {
            lock (readyResults)
            {
                while (readyResults.Count > 0)
                {
                    var newResult = (ReentrySimulation.Result)readyResults.Dequeue();

                    // A queued ordinary result may finish after targeted V1
                    // begins. It has no ownership of the target-aware slot.
                    if (TargetAwareAirlessActive)
                    {
                        LandingPredictorCapture.ResolvedResult(newResult.CaptureSubmissionId,
                            newResult, null, "ordinary_blocked_by_target_aware_mode",
                            Interlocked.Read(ref predictionGeneration),
                            Core.Landing.CurrentStep?.GetType().Name);
                        LandingPredictorCapture.Discarded(newResult.CaptureSubmissionId,
                            "ordinary_cannot_replace_target_aware");
                        newResult.Release();
                        continue;
                    }

                    // If running the simulation resulted in an error then just ignore it.
                    if (newResult.Outcome != ReentrySimulation.Outcome.ERROR)
                    {
                        if (newResult.Body != null)
                            newResult.EndASL = newResult.Body.TerrainAltitude(newResult.EndPosition.Latitude, newResult.EndPosition.Longitude);

                        if (newResult.MultiplierHasError)
                        {
                            LandingPredictorCapture.ResolvedResult(newResult.CaptureSubmissionId, newResult, null,
                                "parachute_error", Interlocked.Read(ref predictionGeneration),
                                Core.Landing.CurrentStep?.GetType().Name);
                            if (errorResult != null)
                            {
                                LandingPredictorCapture.Discarded(errorResult.CaptureSubmissionId,
                                    "error_result_replaced");
                                errorResult.Release();
                            }
                            errorResult = newResult;
                        }
                        else
                        {
                            TerrainProfileTrace terrainTrace = Core.Landing.LandingTraceEnabled
                                ? new TerrainProfileTrace
                                {
                                    SimulatorEndpoint = newResult.EndPosition,
                                    SimulatorEndpointASL = newResult.EndASL
                                }
                                : null;
                            // An airless trajectory is unaffected by terrain until first
                            // contact. Simulate to sea level, then on the flight thread
                            // resolve the first recorded path sample that reaches the
                            // body's actual terrain. This is a terrain profile, not a
                            // single spherical contact height, so ridges cannot move the
                            // next simulation onto a different branch.
                            if (newResult.Outcome == ReentrySimulation.Outcome.LANDED &&
                                newResult.Body != null && !newResult.Body.atmosphere)
                            {
                                ResolveAirlessTerrainProfileContact(newResult, terrainTrace);
                            }
                            LandingPredictorCapture.ResolvedResult(newResult.CaptureSubmissionId, newResult,
                                terrainTrace, "ordinary_processed", Interlocked.Read(ref predictionGeneration),
                                Core.Landing.CurrentStep?.GetType().Name);
                            AcceptNormalResult(newResult, terrainTrace);
                        }
                    }
                    else
                    {
                        LandingPredictorCapture.ResolvedResult(newResult.CaptureSubmissionId, newResult, null,
                            "simulation_error_discarded", Interlocked.Read(ref predictionGeneration),
                            Core.Landing.CurrentStep?.GetType().Name);
                        if (newResult.Exception != null)
                            Print("Exception in the last simulation\n" + newResult.Exception.Message + "\n" + newResult.Exception.StackTrace);
                        newResult.Release();
                    }
                }
            }
        }

        private void ResolveAirlessTerrainProfileContact(ReentrySimulation.Result simulationResult, TerrainProfileTrace trace)
        {
            if (simulationResult.Trajectory == null || simulationResult.Trajectory.Count == 0)
                return;

            // TerrainAltitude is a main-thread query. Limit it to the final
            // portion of the path below the body's highest possible terrain,
            // rather than querying every orbital trajectory sample.
            double maximumTerrainASL = 0;
            if (simulationResult.Body.pqsController != null)
                maximumTerrainASL = Math.Max(0, simulationResult.Body.pqsController.radiusMax - simulationResult.Body.Radius);

            int firstPossibleContact = simulationResult.Trajectory.Count - 1;
            for (int i = 0; i < simulationResult.Trajectory.Count; ++i)
            {
                if (simulationResult.Trajectory[i].Radius - simulationResult.Body.Radius <= maximumTerrainASL)
                {
                    firstPossibleContact = i;
                    break;
                }
            }

            int sampleCount = simulationResult.Trajectory.Count - firstPossibleContact;
            if (trace != null)
            {
                trace.FirstProfileIndex = firstPossibleContact;
                trace.ProfileSampleCount = sampleCount;
            }
            var altitudeASL = new List<double>(sampleCount);
            var terrainASL = new List<double>(sampleCount);
            bool captureTerrain = trace != null && simulationResult.CaptureSubmissionId != 0;
            StringBuilder captureSamples = null;
            if (captureTerrain)
            {
                try { captureSamples = new StringBuilder().Append('['); }
                catch (Exception) { /* Terrain resolution must proceed if capture allocation fails. */ }
            }
            long terrainQueryTicks = 0;
            for (int i = firstPossibleContact; i < simulationResult.Trajectory.Count; ++i)
            {
                AbsoluteVector sample = simulationResult.Trajectory[i];
                double altitude = sample.Radius - simulationResult.Body.Radius;
                long terrainQueryStart = captureTerrain ? Stopwatch.GetTimestamp() : 0;
                double terrain = simulationResult.Body.TerrainAltitude(sample.Latitude, sample.Longitude);
                if (captureTerrain)
                    terrainQueryTicks += Stopwatch.GetTimestamp() - terrainQueryStart;
                altitudeASL.Add(altitude);
                terrainASL.Add(terrain);
                if (captureSamples != null)
                {
                    try
                    {
                        if (i != firstPossibleContact) captureSamples.Append(',');
                        captureSamples.Append('[').Append(LandingPredictorCapture.CaptureNumber(sample.Latitude))
                            .Append(',').Append(LandingPredictorCapture.CaptureNumber(sample.Longitude))
                            .Append(',').Append(LandingPredictorCapture.CaptureNumber(altitude))
                            .Append(',').Append(LandingPredictorCapture.CaptureNumber(terrain)).Append(']');
                    }
                    catch (Exception)
                    {
                        captureSamples = null;
                    }
                }
            }
            if (captureTerrain)
            {
                trace.TerrainQueryCount = sampleCount;
                trace.TerrainQueryElapsedMs = 1000d * terrainQueryTicks / Stopwatch.Frequency;
                try { trace.CaptureSamples = captureSamples?.Append(']').ToString(); }
                catch (Exception) { trace.CaptureSamples = null; }
            }

            int localContactIndex = AirlessTerrainProfileContact.FindFirstContactIndex(altitudeASL, terrainASL);
            if (localContactIndex < 0)
                return;

            if (trace != null)
                trace.LocalContactIndex = localContactIndex;

            int contactIndex = AirlessTerrainProfileContact.ToTrajectoryIndex(firstPossibleContact,
                localContactIndex, simulationResult.Trajectory.Count);
            if (contactIndex < 0)
                return;
            if (trace != null)
                trace.ContactIndex = contactIndex;
            AbsoluteVector contact = simulationResult.Trajectory[contactIndex];
            simulationResult.EndPosition = contact;
            simulationResult.EndUT = contact.UT;
            // contactIndex addresses the complete simulator trajectory. The
            // terrain profile is a slice beginning at firstPossibleContact,
            // so it must be read with its local contact index.
            simulationResult.EndASL = terrainASL[localContactIndex];
            if (trace != null)
            {
                trace.ContactTerrainASL = simulationResult.EndASL;
                trace.Applied = true;
            }
        }

        private double ResultAcceptanceDistance(ReentrySimulation.Result first, ReentrySimulation.Result second)
        {
            if (first.Body == null || first.Body != second.Body)
                return double.PositiveInfinity;

            Vector3d firstPosition = first.Body.GetWorldSurfacePosition(first.EndPosition.Latitude, first.EndPosition.Longitude, 0);
            Vector3d secondPosition = second.Body.GetWorldSurfacePosition(second.EndPosition.Latitude, second.EndPosition.Longitude, 0);
            return Vector3d.Distance(firstPosition, secondPosition);
        }

        private bool ResultsAgree(ReentrySimulation.Result first, ReentrySimulation.Result second)
        {
            double inputTimeDifference = Math.Abs(second.InputUT - first.InputUT);
            double expectedSnapshotMotion = VesselState.SpeedSurface * inputTimeDifference;
            double acceptanceDistance = Math.Min(MaximumResultAcceptanceDistance,
                Math.Max(MinimumResultAcceptanceDistance, 25 + 0.5 * expectedSnapshotMotion));
            return LandingPredictionConsensus.Agrees(first, second, acceptanceDistance,
                ResultAcceptanceDistance(first, second));
        }

        private void TraceNormalResultDecision(string decision, ReentrySimulation.Result comparedResult,
            ReentrySimulation.Result newResult, TerrainProfileTrace terrainTrace)
        {
            if (!Core.Landing.LandingTraceEnabled || !Core.Landing.LandAtTarget)
                return;

            LandingPredictorCapture.Decision(newResult.CaptureSubmissionId, decision,
                comparedResult?.CaptureSubmissionId ?? 0);

            double inputTimeDifference = comparedResult == null ? double.NaN :
                Math.Abs(newResult.InputUT - comparedResult.InputUT);
            double expectedSnapshotMotion = VesselState.SpeedSurface * inputTimeDifference;
            double acceptanceDistance = Math.Min(MaximumResultAcceptanceDistance,
                Math.Max(MinimumResultAcceptanceDistance, 25 + 0.5 * expectedSnapshotMotion));
            double resultDistance = comparedResult == null ? double.NaN :
                ResultAcceptanceDistance(comparedResult, newResult);
            Core.Landing.TraceLanding($"predictor {decision} inputDt={inputTimeDifference:F3} " +
                $"distance={resultDistance:F1} acceptance={acceptanceDistance:F1} " +
                $"newLat={newResult.EndPosition.Latitude:F6} newLon={newResult.EndPosition.Longitude:F6} " +
                $"endUT={newResult.EndUT:F2} inputTerrain={newResult.InputProbableLandingSiteASL:F1} " +
                $"endpointTerrain={newResult.EndASL:F1}");
            Core.Landing.TracePredictorDiagnostic(decision, newResult, terrainTrace, lastSimTime);
        }

        private void PublishNormalResult(ReentrySimulation.Result newResult)
        {
            // Recheck ownership at the final write, after any terrain queries.
            // A mode/target change during resolution cannot grant an ordinary
            // or legacy planner result the target-aware active slot.
            if (TargetAwareAirlessActive)
            {
                LandingPredictorCapture.Decision(newResult.CaptureSubmissionId,
                    "ordinary_blocked_at_publication", result?.CaptureSubmissionId ?? 0);
                LandingPredictorCapture.Discarded(newResult.CaptureSubmissionId,
                    "ordinary_cannot_replace_target_aware");
                newResult.Release();
                return;
            }
            if (result != null)
            {
                LandingPredictorCapture.Discarded(result.CaptureSubmissionId,
                    "published_result_replaced");
                result.Release();
            }

            result = newResult;
            ResultVersion++;
            LandingPredictorCapture.Published(newResult.CaptureSubmissionId, ResultVersion,
                Interlocked.Read(ref predictionGeneration), Core.Landing.CurrentStep?.GetType().Name);
        }

        private void AcceptNormalResult(ReentrySimulation.Result newResult, TerrainProfileTrace terrainTrace)
        {
            // A retained LANDED result is an actuator input for V1.  Never let
            // the two-sample landing consensus conceal a fresh non-landing
            // result: Course Correction must not calculate another pulse from
            // an impact point after the real orbit has stopped intersecting the
            // protected descent radius.
            if (LandingPredictionConsensus.RequiresImmediatePublication(newResult.Outcome))
            {
                TraceNormalResultDecision("immediate_invalidate", candidateResult, newResult, terrainTrace);
                if (candidateResult != null)
                {
                    candidateResult.Release();
                    candidateResult = null;
                }

                PublishNormalResult(newResult);
                return;
            }

            // A result controls both the map marker and V1 course corrections.
            // Publish only a consensus result. In particular, do not let the
            // first result after a trajectory change select one side of a
            // terrain/impact branch before a second simulation corroborates it.
            // Publish only after two consecutive self-consistent simulations
            // agree. Comparing a new result directly with the old published
            // result lets an A/B/A branch sequence accept A on every second
            // frame while B remains unresolved, which is exactly the visible
            // marker and course-correction oscillation reported on Minmus.
            bool candidateAgrees = candidateResult != null && ResultsAgree(candidateResult, newResult);
            if (LandingPredictionTerrainConvergence.HasConsecutiveAgreement(candidateResult != null, candidateAgrees))
            {
                TraceNormalResultDecision(result == null ? "initial_accept" : "candidate_accept", candidateResult, newResult, terrainTrace);
                candidateResult.Release();
                candidateResult = null;
                PublishNormalResult(newResult);
                return;
            }

            TraceNormalResultDecision(candidateResult == null ? "initial_pending" : "replace", candidateResult, newResult, terrainTrace);
            if (candidateResult != null)
                candidateResult.Release();
            candidateResult = newResult;
        }

        protected Orbit GetReenteringPatch()
        {
            Orbit patch = Orbit;

            int i = 0;

            do
            {
                i++;
                double reentryRadius = patch.referenceBody.Radius + patch.referenceBody.RealMaxAtmosphereAltitude();
                Orbit nextPatch = Vessel.GetNextPatch(patch, aerobrakeNode);
                if (patch.PeR < reentryRadius)
                {
                    if (patch.Radius(patch.StartUT) < reentryRadius) return patch;

                    double reentryTime = patch.NextTimeOfRadius(patch.StartUT, reentryRadius);
                    if (patch.StartUT < reentryTime && (nextPatch == null || reentryTime < nextPatch.StartUT))
                    {
                        return patch;
                    }
                }

                patch = nextPatch;
            } while (patch != null);

            return null;
        }

        protected void MaintainAerobrakeNode()
        {
            if (makeAerobrakeNodes)
            {
                //Remove node after finishing aerobraking:
                if (aerobrakeNode != null && Vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
                {
                    if (aerobrakeNode.UT < VesselState.Time && VesselState.AltitudeASL > MainBody.RealMaxAtmosphereAltitude())
                    {
                        aerobrakeNode.RemoveSelf();
                        aerobrakeNode = null;
                    }
                }

                //Update or create node if necessary:
                ReentrySimulation.Result r = Result;
                if (r != null && r.Outcome == ReentrySimulation.Outcome.AEROBRAKED)
                {
                    //Compute the node dV:
                    Orbit preAerobrakeOrbit = GetReenteringPatch();

                    //Put the node at periapsis, unless we're past periapsis. In that case put the node at the current time.
                    double UT;
                    if (preAerobrakeOrbit == Orbit &&
                        VesselState.AltitudeASL < MainBody.RealMaxAtmosphereAltitude() && VesselState.SpeedVertical > 0)
                    {
                        UT = VesselState.Time;
                    }
                    else
                    {
                        UT = preAerobrakeOrbit.NextPeriapsisTime(preAerobrakeOrbit.StartUT);
                    }

                    Orbit postAerobrakeOrbit =
                        MuUtils.OrbitFromStateVectors(r.WorldAeroBrakePosition(), r.WorldAeroBrakeVelocity(), r.Body, r.AeroBrakeUT);

                    Vector3d dV = OrbitalManeuverCalculator.DeltaVToChangeApoapsis(preAerobrakeOrbit, UT, postAerobrakeOrbit.ApR);

                    if (aerobrakeNode != null && Vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
                    {
                        //update the existing node
                        Vector3d nodeDV = preAerobrakeOrbit.DeltaVToManeuverNodeCoordinates(UT, dV);
                        aerobrakeNode.UpdateNode(nodeDV, UT);
                    }
                    else
                    {
                        //place a new node
                        aerobrakeNode = Vessel.PlaceManeuverNode(preAerobrakeOrbit, dV, UT);
                    }
                }
                else
                {
                    //no aerobraking, remove the node:
                    if (aerobrakeNode != null && Vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
                    {
                        aerobrakeNode.RemoveSelf();
                    }
                }
            }
            else
            {
                //Remove aerobrake node when it is turned off:
                if (aerobrakeNode != null && Vessel.patchedConicSolver.maneuverNodes.Contains(aerobrakeNode))
                {
                    aerobrakeNode.RemoveSelf();
                }
            }
        }

        private void DoMapView()
        {
            if (predictorDestroyed) return;
            bool conditional = TargetAwareForecastPresentation.ConditionalBeforeDeorbit(
                TargetAwareAirlessActive, Core.Landing.CurrentStep is LowDeorbitBurn ||
                    Core.Landing.CurrentStep is DeorbitBurn, observedDeorbitThrust);
            if (Core.Landing.LandingTraceEnabled)
            {
                int flags = (MapView.MapIsEnabled ? 1 : 0) | (camTrajectory ? 2 : 0) |
                    (Enabled ? 4 : 0) | (Vessel.LandedOrSplashed ? 8 : 0) |
                    (Result != null ? 16 : 0) | (result != null ? 32 : 0) | (conditional ? 64 : 0) |
                    (Result?.Outcome == ReentrySimulation.Outcome.TIMED_OUT ? 128 : 0);
                if (lastMapCaptureVersion != ResultVersion || lastMapCaptureFlags != flags)
                {
                    lastMapCaptureVersion = ResultVersion;
                    lastMapCaptureFlags = flags;
                    string reason = (flags & 3) == 0 ? "view_disabled" : !Enabled ? "predictor_disabled" :
                        Vessel.LandedOrSplashed ? "vessel_landed" : result == null ? "no_committed_result" :
                        Result == null ? "forecast_unavailable_or_settling" :
                        conditional ? "conditional_current_trajectory_before_deorbit" :
                        Result.Outcome != ReentrySimulation.Outcome.LANDED ? "not_predicted_touchdown" :
                        "marker_requested_occlusion_not_observed";
                    LandingPredictorCapture.MapDraw(ResultVersion, Result?.CaptureSubmissionId ?? result?.CaptureSubmissionId ?? 0,
                        MapView.MapIsEnabled, camTrajectory, Enabled, Vessel.LandedOrSplashed,
                        Result?.Outcome.ToString(), !conditional && (flags & 3) != 0 && (flags & 12) == 4 &&
                            Result?.Outcome == ReentrySimulation.Outcome.LANDED,
                        reason, Core.Landing.CurrentStep?.GetType().Name);
                }
            }
            if ((MapView.MapIsEnabled || camTrajectory) && !Vessel.LandedOrSplashed && Enabled)
            {
                ReentrySimulation.Result drawnResult = PresentedForecast;
                bool previousForecast = !PresentedForecastCurrent;
                if (drawnResult != null)
                {
                    if (drawnResult.Outcome == ReentrySimulation.Outcome.LANDED)
                        GLUtils.DrawGroundMarker(drawnResult.Body, drawnResult.EndPosition.Latitude, drawnResult.EndPosition.Longitude, conditional || previousForecast ? Color.gray : Color.blue,
                            MapView.MapIsEnabled, 60, 0, drawnResult.EndASL);

                    if (showTrajectory && drawnResult.Trajectory != null &&
                        drawnResult.Outcome != ReentrySimulation.Outcome.ERROR &&
                        drawnResult.Outcome != ReentrySimulation.Outcome.NO_REENTRY)
                    {
                        double interval = Math.Max(Math.Min((drawnResult.EndUT - drawnResult.InputUT) / 1000, 10), 0.1);
                        if (drawnResult.HasControllerBrakeReferenceUT)
                        {
                            // A target-aware result includes a long coast. Draw
                            // that separately so the red path means actual
                            // predicted braking, not the entire flight.
                            using (Disposable<List<Vector3d>> coast = drawnResult.WorldTrajectorySegment(
                                interval, worldTrajectory, drawnResult.InputUT,
                                drawnResult.SimulatedBrakingStartUT))
                                if (coast.value.Count > 1)
                                    GLUtils.DrawPath(drawnResult.Body, coast.value, conditional || previousForecast ? Color.gray : Color.cyan,
                                        MapView.MapIsEnabled);
                            using (Disposable<List<Vector3d>> burn = drawnResult.WorldTrajectorySegment(
                                interval, worldTrajectory, drawnResult.SimulatedBrakingStartUT,
                                drawnResult.EndUT))
                                if (burn.value.Count > 1)
                                    GLUtils.DrawPath(drawnResult.Body, burn.value, conditional || previousForecast ? Color.gray : Color.red,
                                        MapView.MapIsEnabled);
                        }
                        else using (Disposable<List<Vector3d>> list = drawnResult.WorldTrajectory(
                            interval, worldTrajectory))
                        {
                            if (!MapView.MapIsEnabled && (noSkipToFreefall || Vessel.staticPressurekPa > 0))
                                list.value[0] = VesselState.CoM;
                            GLUtils.DrawPath(drawnResult.Body, list.value, conditional || previousForecast ? Color.gray : Color.red, MapView.MapIsEnabled);
                        }
                    }
                }
            }
        }

        public MechJebModuleLandingPredictions(MechJebCore core) : base(core) { }
    }
}
