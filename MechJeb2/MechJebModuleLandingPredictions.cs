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
    public class MechJebModuleLandingPredictions : ComputerModule
    {
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
            result;

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
        private readonly Queue readyBrakingPlanResults = new Queue();
        private readonly Queue<TargetAwareWorkCompletion> readyTargetAwareWork =
            new Queue<TargetAwareWorkCompletion>();
        private TargetAwareAirlessPlanner activeTargetAware;
        private Orbit activeTargetAwareOrbit;
        private long activeTargetAwareCaptureId;
        private TargetAwareResultLineage? committedTargetAware;
        private bool wasTargetAwareActive;
        private long targetAwareSequence;
        private double lastTargetAwareStartUT = double.NegativeInfinity;
        private double targetAwareWorkerMilliseconds;
        private const int TargetAwareTerrainQueriesPerTick = 32;
        // Provisional safety cap. Mun replay measures the actual queries before
        // this becomes an acceptance budget; exceeding it fails the candidate.
        private const int TargetAwareMaximumTerrainQueries = 1024;
        private int brakingPlanRunning;
        private double lastBrakingPlanInputUT = double.NegativeInfinity;
        private const int BrakingPlanCandidateCount = 9;
        private const double BrakingPlanRefreshSeconds = 5;
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
            wasTargetAwareActive = false;
            Interlocked.Increment(ref predictionGeneration);
            if (Core.Landing.LandingTraceEnabled && Core.Landing.LandAtTarget &&
                VesselState != null && VesselState.MainBody != null && !VesselState.MainBody.atmosphere)
                LandingPredictorCapture.Lifecycle("predictor_disabled", Interlocked.Read(ref predictionGeneration),
                    ResultVersion, result?.CaptureSubmissionId ?? 0);
            ReleaseQueuedBrakingPlanResults();
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

        // A targeted V1 landing is a new predictor transaction. A result or
        // terrain radius from a prior target must never command its first
        // course correction. The landing autopilot calls this before it starts
        // its first phase; no actuator state is changed here.
        public void ResetTargetedLandingPrediction()
        {
            Interlocked.Increment(ref predictionGeneration);
            if (Core.Landing.LandingTraceEnabled && Core.Landing.LandAtTarget &&
                VesselState != null && VesselState.MainBody != null && !VesselState.MainBody.atmosphere)
                LandingPredictorCapture.Lifecycle("target_reset", Interlocked.Read(ref predictionGeneration),
                    ResultVersion, result?.CaptureSubmissionId ?? 0);
            ReleaseQueuedBrakingPlanResults();
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

        public override void OnFixedUpdate()
        {
            bool targetAware = TargetAwareAirlessActive;
            if (targetAware && !wasTargetAwareActive)
            {
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
            wasTargetAwareActive = targetAware;
            InvalidateChangedTargetAwareContext();
            if (TargetAwareAirlessActive && IsTerminalLandingPhase &&
                activeTargetAware != null &&
                (!activeTargetAware.IsDirectForecast ||
                 !(Core.Landing.CurrentStep is DecelerationBurn)))
                CancelTargetAwareWork();
            CheckForResult();
            CheckForBrakingPlanResult();
            CheckForTargetAwareWork();
            AdvanceTargetAwareTransaction();
            if (TargetAwareAirlessActive)
            {
                if (Core.Landing.CurrentStep is CourseCorrection ||
                    Core.Landing.CurrentStep is CoastToDeceleration ||
                    Core.Landing.CurrentStep is DecelerationBurn)
                    TryStartTargetAwareTransaction();
            }
            else
                TryStartSimulation(true);
        }

        private bool IsTerminalLandingPhase =>
            Core.Landing.CurrentStep is DecelerationBurn ||
            Core.Landing.CurrentStep is KillHorizontalVelocity ||
            Core.Landing.CurrentStep is FinalDescent;

        private void InvalidateCommittedTargetAware(string reason)
        {
            if (!committedTargetAware.HasValue) return;
            if (Core.Landing.LandingTraceEnabled)
                LandingPredictorCapture.Lifecycle(reason, Interlocked.Read(ref predictionGeneration),
                    ResultVersion, result?.CaptureSubmissionId ?? 0);
            if (result != null)
            {
                LandingPredictorCapture.Discarded(result.CaptureSubmissionId, reason);
                result.Release();
                result = null;
            }
            committedTargetAware = null;
            lastTargetAwareStartUT = double.NegativeInfinity;
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

        private enum TargetAwareWorkKind { Ballistic, Coarse, Refinement, DirectForecast }

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
            public Exception Exception;
            public double ElapsedMilliseconds;
        }

        private void CancelTargetAwareWork()
        {
            if (activeTargetAwareCaptureId != 0)
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

        private void TryStartTargetAwareTransaction()
        {
            if (activeTargetAware != null || Vessel.LandedOrSplashed)
                return;
            bool directForecast = Core.Landing.CurrentStep is DecelerationBurn;
            if (directForecast &&
                !((DecelerationBurn)Core.Landing.CurrentStep).BrakingTriggered)
                return;
            double now = Planetarium.GetUniversalTime();
            if (Core.Landing.CurrentStep is CourseCorrection correction &&
                !correction.PredictionSnapshotSafe(now))
                return;
            if (now - lastTargetAwareStartUT <
                (directForecast ? 2.0 : BrakingPlanRefreshSeconds))
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
                // V1's airless predictor policy uses target terrain + 200 m.
                // Snapshot that same value here so update order cannot import
                // an unset or prior-target mutable policy field.
                double activeDecelEndASL = directForecast ?
                    Core.Landing.DecelerationEndAltitude() : targetTerrainASL + 200;
                double maximumThrustAcceleration = VesselState.LimitedMaxThrustAcceleration;
                var activePolicy = new SafeDescentSpeedPolicy(body.Radius + activeDecelEndASL,
                    body.GeeASL * 9.81, maximumThrustAcceleration);
                if (committedTargetAware.HasValue &&
                    committedTargetAware.Value.TargetTerrainASL != targetTerrainASL)
                    InvalidateCommittedTargetAware("target_aware_target_terrain_changed");
                var rawSnapshot = new AirlessTargetAwareSnapshot(inputUT, now,
                    body.Radius, body.gravParameter, body.GeeASL, body.rotationPeriod,
                    Core.Target.targetLatitude, Core.Target.targetLongitude, targetTerrainASL,
                    activeDecelEndASL, maximumThrustAcceleration, dt,
                    Time.fixedDeltaTime, maxOrbits,
                    body.pqsController.radiusMin - body.Radius,
                    body.pqsController.radiusMax - body.Radius,
                    patch.WorldBCIPositionAtUT(inputUT), patch.WorldOrbitalVelocityAtUT(inputUT),
                    body.angularVelocity, body.GetSurfaceNVector(0, 0),
                    body.GetSurfaceNVector(0, 90), body.GetSurfaceNVector(90, 0));
                if (!(Core.Landing.DescentSpeedPolicy is SafeDescentSpeedPolicy v1Policy))
                    throw new InvalidOperationException("V1 airless speed policy unavailable");
                var snapshot = new AirlessTargetAwareSnapshot(rawSnapshot,
                    VesselState.Mass, VesselState.ThrustAvailable,
                    VesselState.ThrustMinimum, VesselState.MaximumEngineMassFlow,
                    VesselState.MinimumEngineMassFlow,
                    directForecast ? v1Policy.TerrainRadius :
                    body.Radius + targetTerrainASL + 200,
                    v1Policy.Gravity, v1Policy.Thrust, directForecast,
                    VesselState.Forward,
                    Core.Thrust.LimiterMinThrottle ? (double)Core.Thrust.MinThrottle : 0,
                    Math.Max(Core.Thrust.LimiterMinThrottle ?
                        (double)Core.Thrust.MinThrottle : 0,
                        directForecast ? Core.Thrust.ThrottleLimit :
                            Core.Thrust.ThrottleFixedLimit),
                    Core.Thrust.SmoothThrottle ? Core.Thrust.ThrottleSmoothingTime : 0,
                    directForecast ? Core.Thrust.LastThrottle : 0);
                var orbitCopy = new Orbit();
                orbitCopy.UpdateFromOrbitAtUT(patch, inputUT, body);
                var planner = new TargetAwareAirlessPlanner(snapshot,
                    Interlocked.Read(ref predictionGeneration), ++targetAwareSequence, body,
                    (latitude, longitude) => body.TerrainAltitude(latitude, longitude),
                    Math.Max(200, body.Radius * 0.0005), TargetAwareTerrainQueriesPerTick,
                    TargetAwareMaximumTerrainQueries, 1, directForecast);
                activeTargetAware = planner;
                activeTargetAwareOrbit = orbitCopy;
                targetAwareWorkerMilliseconds = 0;
                var activeTargetAwareCapture = CaptureSnapshot(patch, inputUT, body, snapshot,
                    targetTerrainTimer.Elapsed.TotalMilliseconds);
                activeTargetAwareCaptureId = LandingPredictorCapture.Submit(activeTargetAwareCapture,
                    "target_aware_transaction", activePolicy, activeDecelEndASL,
                    targetTerrainASL, maximumThrustAcceleration,
                    parachuteSemiDeployMultiplier, dt, Time.fixedDeltaTime, maxOrbits,
                    noSkipToFreefall, double.NaN,
                    directForecast ? "v1_live_braking_forecast" :
                    "v1_controller_policy_forecast");
                LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                    "target_aware_refresh_started", result?.CaptureSubmissionId ?? 0);
                lastTargetAwareStartUT = now;
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
            if (!ThreadPool.QueueUserWorkItem(RunTargetAwareWork,
                    new TargetAwareWorkItem(planner, kind, activeTargetAwareCaptureId)))
                planner.WorkerFailed("WorkerQueueRejected:" + kind);
        }

        private void RunTargetAwareWork(object state)
        {
            var item = (TargetAwareWorkItem)state;
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
                    case TargetAwareWorkKind.DirectForecast:
                        completion.Revalidated = AirlessTargetAwareSimulation.Run(
                            item.Planner.Snapshot, item.Planner.Snapshot.InputUT, true);
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
                    int outputCount = work.Kind == TargetAwareWorkKind.DirectForecast ?
                        (work.Revalidated == null ? 0 : 1) :
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
                            work.Exception.GetType().Name);
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
                            case TargetAwareWorkKind.DirectForecast:
                                work.Planner.SetDirectOutput(work.Revalidated);
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
            if (age < 0 || age > 2 * BrakingPlanRefreshSeconds)
                planner.WorkerFailed("SnapshotExpiredBeforePublication");
            if (planner.Stage == TargetAwarePlannerStage.ResolveBallistic ||
                planner.Stage == TargetAwarePlannerStage.ResolveCoarse ||
                planner.Stage == TargetAwarePlannerStage.ResolveRefinement ||
                planner.Stage == TargetAwarePlannerStage.ResolveDirectForecast ||
                planner.Stage == TargetAwarePlannerStage.ResolveSelectedCoast)
                planner.AdvanceTerrain();

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
            else if (planner.Stage == TargetAwarePlannerStage.Complete)
            {
                TryPublishTargetAwareResult(planner);
                activeTargetAware = null;
                activeTargetAwareOrbit = null;
                activeTargetAwareCaptureId = 0;
            }
            else if (planner.Stage == TargetAwarePlannerStage.Failed)
            {
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
                var body = (CelestialBody)planner.BodyIdentity;
                AirlessTargetAwareSnapshot snapshot = planner.Snapshot;
                if (!snapshot.HasV1ControlModel || planner.SelectedOutput == null ||
                    !planner.SelectedOutput.UsesV1ControlModel)
                    throw new InvalidOperationException("Non-V1 model cannot publish");
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
                    true, planner.SelectedOutput != null && planner.SelectedOutput.ReachedHandoff,
                    planner.SelectedTerrain != null && planner.SelectedTerrain.Resolved,
                    planner.SelectedTerrain != null && planner.SelectedTerrain.ClearPath,
                    planner.TerminalHandoff.NecessaryControlBoundPasses);
                TargetAwarePublicationDecision decision = TargetAwarePublicationGate.Check(lineage,
                    committedTargetAware, Interlocked.Read(ref predictionGeneration),
                    Core.Target.targetBody, Core.Target.targetLatitude, Core.Target.targetLongitude,
                    currentTerrain, Planetarium.GetUniversalTime(),
                    2 * BrakingPlanRefreshSeconds, 0);
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

                replacement = BuildTargetAwareResult(planner, body);
                LandingPredictorCapture.WorkerResult(activeTargetAwareCaptureId, replacement,
                    false, targetAwareWorkerMilliseconds);
                LandingPredictorCapture.ResolvedResult(activeTargetAwareCaptureId, replacement,
                    null, "target_aware_clearance_validated", planner.Generation,
                    Core.Landing.CurrentStep?.GetType().Name);
                LandingPredictorCapture.Decision(activeTargetAwareCaptureId,
                    "target_aware_atomic_accept", result?.CaptureSubmissionId ?? 0);
                ReentrySimulation.Result predecessor = result;
                result = replacement;
                replacement = null;
                committedTargetAware = lineage;
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
                        $"vertical={planner.TerminalHandoff.EndVerticalSpeed:F1} " +
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
                published.Outcome = ReentrySimulation.Outcome.LANDED;
                published.Exception = null;
                published.Body = body;
                published.ReferenceFrame = new ReferenceFrame();
                published.ReferenceFrame.UpdateAtCurrentTime(body);
                published.ID = unchecked((ulong)planner.Sequence);
                published.InputInitialOrbit = activeTargetAwareOrbit;
                published.InputUT = snapshot.InputUT;
                published.InputDescentSpeedPolicy = new SafeDescentSpeedPolicy(
                    body.Radius + snapshot.TargetTerrainASL + 200,
                    snapshot.BodyGeeASL * 9.81, snapshot.MaximumThrustAcceleration);
                published.InputDecelEndAltitudeASL =
                    snapshot.TargetTerrainASL + 200;
                published.InputMaxThrustAccel = snapshot.MaximumThrustAcceleration;
                published.InputProbableLandingSiteASL = snapshot.TargetTerrainASL;
                published.InputParachuteSemiDeployMultiplier = 0;
                published.InputMultiplierHasError = false;
                published.InputDT = snapshot.Dt;
                published.InputMaxOrbits = snapshot.MaxOrbits;
                published.InputNoSkipToFreefall = false;
                published.InputForcedBrakingStartUT = output.BrakeUT;
                published.InputParachuteList = null;
                published.MultiplierHasError = false;
                published.ParachuteMultiplier = 0;
                published.AeroBrake = false;
                published.AeroBrakeUT = 0;
                published.AeroBrakePosition = default(AbsoluteVector);
                published.AeroBrakeVelocity = default(AbsoluteVector);
                published.DebugLog = null;
                published.SimulatedBrakingStartUT = output.Trajectory[0].UT;
                published.HasControllerBrakeReferenceUT = true;
                published.ControllerBrakeReferenceUT = output.BrakeUT;
                AirlessTargetAwareState reference = output.Trajectory[0];
                foreach (AirlessTargetAwareState state in output.Trajectory)
                {
                    if (state.UT > output.BrakeUT) break;
                    reference = state;
                }
                published.ControllerBrakeReferencePosition =
                    AirlessTargetAwareSimulation.ToAbsolute(reference.Position,
                        reference.UT, snapshot);
                published.StartPosition = AirlessTargetAwareSimulation.ToAbsolute(
                    snapshot.Position, snapshot.InputUT, snapshot);
                published.EndPosition = AirlessTargetAwareSimulation.ToAbsolute(
                    output.End.Position, output.End.UT, snapshot);
                published.EndVelocity = AirlessTargetAwareSimulation.ToAbsolute(
                    output.End.Velocity, output.End.UT, snapshot);
                published.EndUT = output.End.UT;
                published.EndASL = planner.SelectedTerrain.LocalTerrainASL;
                published.TimeToComplete = output.End.UT - snapshot.InputUT;
                published.EndSurfaceSpeed = output.EndSurfaceSpeed;
                published.DeltaVExpended = output.VirtualDeltaV;
                published.MaxDragGees = 0;
                published.Maxdt = snapshot.Dt;
                published.Steps = output.Steps;
                published.Prediction = default(ReentrySimulation.Prediction);
                published.CaptureSubmissionId = activeTargetAwareCaptureId;
                published.Trajectory = new List<AbsoluteVector>(
                    output.CoastSamples.Count + output.Trajectory.Count + 1);
                published.Trajectory.Add(published.StartPosition);
                foreach (AirlessTargetAwareState state in output.CoastSamples)
                    published.Trajectory.Add(AirlessTargetAwareSimulation.ToAbsolute(
                        state.Position, state.UT, snapshot));
                foreach (AirlessTargetAwareState state in output.Trajectory)
                    published.Trajectory.Add(AirlessTargetAwareSimulation.ToAbsolute(
                        state.Position, state.UT, snapshot));
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
                            StartTargetAwareBrakingPlan(newResult);
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

        // The legacy predictor selected virtual braking from a scalar safe-speed
        // rule.  In a shallow airless orbit that can be "now" even though a later
        // powered endpoint reaches the selected site.  Evaluate a small, bounded
        // set of starts using the same ReentrySimulation and select the safe
        // powered endpoint nearest the target.  This only changes prediction data;
        // V1's existing steps remain the actuator owner.
        private void StartTargetAwareBrakingPlan(ReentrySimulation.Result source)
        {
            if (!Core.Landing.Enabled || !Core.Landing.LandAtTarget || !Core.Target.PositionTargetExists ||
                source == null || source.Body == null || source.Body.atmosphere ||
                source.Outcome != ReentrySimulation.Outcome.LANDED || descentSpeedPolicy == null ||
                Interlocked.CompareExchange(ref brakingPlanRunning, 1, 0) != 0 ||
                source.InputUT - lastBrakingPlanInputUT < BrakingPlanRefreshSeconds)
                return;

            double earliestUT = Math.Max(source.InputUT, source.SimulatedBrakingStartUT);
            double ballisticImpactUT = BallisticTargetRadiusImpactUT(source);
            // Do not use source.EndUT here. It is the endpoint of the very
            // immediate virtual burn that this search is meant to replace. The
            // search must extend along the unpowered trajectory almost to its
            // target-height surface intersection.
            double latestUT = ballisticImpactUT - 2;
            if (double.IsNaN(earliestUT) || double.IsNaN(latestUT) || latestUT - earliestUT < 4)
            {
                Interlocked.Exchange(ref brakingPlanRunning, 0);
                return;
            }

            var captureIds = new List<long>(BrakingPlanCandidateCount);
            try
            {
                if (Core.Landing.LandingTraceEnabled)
                    Core.Landing.TraceLanding($"predictor target-aware window earliest={earliestUT:F2} ballisticImpact={ballisticImpactUT:F2} latest={latestUT:F2} candidates={BrakingPlanCandidateCount}");
                var simulations = new List<ReentrySimulation>(BrakingPlanCandidateCount);
                LandingPredictorCapture.Snapshot capture = CaptureSnapshot(source.InputInitialOrbit, source.InputUT, source.Body);
                for (int i = 0; i < BrakingPlanCandidateCount; ++i)
                {
                    double fraction = i / (double)(BrakingPlanCandidateCount - 1);
                    double forcedStartUT = earliestUT + fraction * (latestUT - earliestUT);
                    var simCurves = ReentrySimulation.SimCurves.Borrow(source.Body);
                    var simVessel = SimulatedVessel.Borrow(Vessel, simCurves, source.InputUT,
                        Core.Landing.Enabled && deployChutes ? limitChutesStage : -1);
                    double probableLandingSiteASL = Core.Landing.PredictorLandingAltitudeASL();
                    simulations.Add(ReentrySimulation.Borrow(source.InputInitialOrbit, source.InputUT, simVessel,
                        simCurves, descentSpeedPolicy, decelEndAltitudeASL, VesselState.LimitedMaxThrustAcceleration,
                        parachuteSemiDeployMultiplier, probableLandingSiteASL, false, dt,
                        Time.fixedDeltaTime, maxOrbits, noSkipToFreefall, forcedStartUT));
                    captureIds.Add(LandingPredictorCapture.Submit(capture, "forced_candidate", descentSpeedPolicy,
                        decelEndAltitudeASL, probableLandingSiteASL, VesselState.LimitedMaxThrustAcceleration,
                        parachuteSemiDeployMultiplier, dt, Time.fixedDeltaTime, maxOrbits, noSkipToFreefall,
                        forcedStartUT));
                }

                lastBrakingPlanInputUT = source.InputUT;
                ThreadPool.QueueUserWorkItem(RunTargetAwareBrakingPlan,
                    new BrakingPlanJob(simulations, Interlocked.Read(ref predictionGeneration), captureIds));
            }
            catch (Exception ex)
            {
                foreach (long id in captureIds)
                    LandingPredictorCapture.Abandoned(id, "plan_submission_failed");
                Interlocked.Exchange(ref brakingPlanRunning, 0);
                Debug.LogException(ex);
            }
        }

        private void RunTargetAwareBrakingPlan(object state)
        {
            var job = (BrakingPlanJob)state;
            var results = new List<ReentrySimulation.Result>(job.Simulations.Count);
            int completed = 0;
            bool simulationRunning = false;
            try
            {
                for (int i = 0; i < job.Simulations.Count; ++i)
                {
                    ReentrySimulation simulation = job.Simulations[i];
                    Stopwatch captureWorkerTimer = job.CaptureIds[i] != 0 ? Stopwatch.StartNew() : null;
                    simulationRunning = true;
                    ReentrySimulation.Result candidate = simulation.RunSimulation();
                    captureWorkerTimer?.Stop();
                    candidate.CaptureSubmissionId = job.CaptureIds[i];
                    LandingPredictorCapture.WorkerResult(job.CaptureIds[i], candidate,
                        job.Generation != Interlocked.Read(ref predictionGeneration),
                        captureWorkerTimer?.Elapsed.TotalMilliseconds ?? 0);
                    completed = i + 1;
                    simulationRunning = false;
                    simulation.Release();
                    results.Add(candidate);
                }

                if (job.Generation != Interlocked.Read(ref predictionGeneration))
                {
                    foreach (ReentrySimulation.Result candidate in results)
                    {
                        LandingPredictorCapture.Discarded(candidate.CaptureSubmissionId,
                            "stale_plan_worker_generation");
                        candidate.Release();
                    }
                    results = null;
                    return;
                }

                lock (readyBrakingPlanResults)
                    readyBrakingPlanResults.Enqueue(new BrakingPlanResultSet(job.Generation, results));
                results = null;
            }
            catch (Exception ex)
            {
                if (results != null)
                    foreach (ReentrySimulation.Result completedResult in results)
                        LandingPredictorCapture.Discarded(completedResult.CaptureSubmissionId,
                            "plan_worker_or_queue_exception");
                if (simulationRunning && completed < job.CaptureIds.Count)
                    LandingPredictorCapture.WorkerException(job.CaptureIds[completed], ex);
                for (int i = completed + (simulationRunning ? 1 : 0); i < job.CaptureIds.Count; ++i)
                {
                    LandingPredictorCapture.Abandoned(job.CaptureIds[i], "plan_worker_stopped");
                }
                Dispatcher.InvokeAsync(() => Debug.LogException(ex));
            }
            finally
            {
                if (results != null)
                    foreach (ReentrySimulation.Result candidate in results)
                        candidate.Release();
                Interlocked.Exchange(ref brakingPlanRunning, 0);
            }
        }

        private void CheckForBrakingPlanResult()
        {
            lock (readyBrakingPlanResults)
            {
                while (readyBrakingPlanResults.Count > 0)
                {
                    var set = (BrakingPlanResultSet)readyBrakingPlanResults.Dequeue();
                    if (TargetAwareAirlessActive)
                    {
                        ReleaseBrakingPlanResults(set.Results);
                        continue;
                    }
                    if (set.Generation != Interlocked.Read(ref predictionGeneration) || !Core.Target.PositionTargetExists)
                    {
                        foreach (ReentrySimulation.Result candidate in set.Results)
                            LandingPredictorCapture.ResolvedResult(candidate.CaptureSubmissionId, candidate, null,
                                "stale_plan_discarded", Interlocked.Read(ref predictionGeneration),
                                Core.Landing.CurrentStep?.GetType().Name);
                        ReleaseBrakingPlanResults(set.Results);
                        continue;
                    }

                    var candidates = new List<TargetAwareBrakingPlan.Candidate>(set.Results.Count);
                    foreach (ReentrySimulation.Result candidate in set.Results)
                    {
                        double error = TargetDistance(candidate);
                        if (Core.Landing.LandingTraceEnabled)
                            Core.Landing.TracePredictorDiagnostic("target_aware_candidate", candidate, null, lastSimTime);

                        // The simulator's virtual-braking envelope ends at the
                        // V1 final-descent handoff, not at touchdown. Its terminal
                        // surface speed is diagnostic evidence, not a feasibility
                        // gate. Treating it as a hard reject prevented every
                        // target-aware candidate from ever being published.
                        bool safe = candidate.Outcome == ReentrySimulation.Outcome.LANDED &&
                            !double.IsNaN(error) && !double.IsInfinity(error);
                        candidates.Add(new TargetAwareBrakingPlan.Candidate(candidate.SimulatedBrakingStartUT, error, 0, safe));
                    }

                    if (!TargetAwareBrakingPlan.TrySelect(candidates, out TargetAwareBrakingPlan.Candidate selected))
                    {
                        foreach (ReentrySimulation.Result candidate in set.Results)
                            LandingPredictorCapture.ResolvedResult(candidate.CaptureSubmissionId, candidate, null,
                                "plan_rejected", Interlocked.Read(ref predictionGeneration),
                                Core.Landing.CurrentStep?.GetType().Name);
                        if (Core.Landing.LandingTraceEnabled)
                            Core.Landing.TraceLanding($"predictor target-aware rejected all candidates count={set.Results.Count}");
                        ReleaseBrakingPlanResults(set.Results);
                        continue;
                    }

                    ReentrySimulation.Result selectedResult = null;
                    foreach (ReentrySimulation.Result candidate in set.Results)
                    {
                        if (candidate.SimulatedBrakingStartUT == selected.StartUT)
                        {
                            selectedResult = candidate;
                            break;
                        }
                    }
                    if (selectedResult == null)
                    {
                        foreach (ReentrySimulation.Result candidate in set.Results)
                            LandingPredictorCapture.ResolvedResult(candidate.CaptureSubmissionId, candidate, null,
                                "plan_selection_missing", Interlocked.Read(ref predictionGeneration),
                                Core.Landing.CurrentStep?.GetType().Name);
                        ReleaseBrakingPlanResults(set.Results);
                        continue;
                    }

                    // Terrain is a Unity main-thread query. Resolve it only for the
                    // selected candidate so plan evaluation remains bounded and does
                    // not create the game pauses seen in earlier V2 experiments.
                    TerrainProfileTrace selectedTerrain = selectedResult.CaptureSubmissionId != 0
                        ? new TerrainProfileTrace() : null;
                    if (!selectedResult.Body.atmosphere)
                        ResolveAirlessTerrainProfileContact(selectedResult, selectedTerrain);
                    LandingPredictorCapture.ResolvedResult(selectedResult.CaptureSubmissionId, selectedResult,
                        selectedTerrain, "selected_for_publication", Interlocked.Read(ref predictionGeneration),
                        Core.Landing.CurrentStep?.GetType().Name);
                    PublishNormalResult(selectedResult);
                    if (Core.Landing.LandingTraceEnabled)
                        Core.Landing.TraceLanding($"predictor target-aware brakeStart={selected.StartUT:F2} targetError={selected.DownrangeError:F1} candidates={set.Results.Count}");

                    foreach (ReentrySimulation.Result candidate in set.Results)
                        if (candidate != selectedResult)
                        {
                            LandingPredictorCapture.ResolvedResult(candidate.CaptureSubmissionId, candidate, null,
                                "candidate_not_selected", Interlocked.Read(ref predictionGeneration),
                                Core.Landing.CurrentStep?.GetType().Name);
                            candidate.Release();
                        }
                }
            }
        }

        private double TargetDistance(ReentrySimulation.Result candidate)
        {
            if (candidate.Body == null || candidate.Body != Core.Target.targetBody)
                return double.NaN;
            Vector3d endpoint = candidate.Body.GetWorldSurfacePosition(candidate.EndPosition.Latitude,
                candidate.EndPosition.Longitude, 0);
            Vector3d target = candidate.Body.GetWorldSurfacePosition(Core.Target.targetLatitude,
                Core.Target.targetLongitude, 0);
            return Vector3d.Distance(endpoint, target);
        }

        private double BallisticTargetRadiusImpactUT(ReentrySimulation.Result source)
        {
            try
            {
                double targetRadius = source.Body.Radius + Core.Landing.PredictorLandingAltitudeASL();
                if (source.InputInitialOrbit.PeR >= targetRadius)
                    return double.NaN;

                double impactUT = source.InputInitialOrbit.NextTimeOfRadius(source.InputUT, targetRadius);
                return double.IsInfinity(impactUT) || double.IsNaN(impactUT) ? double.NaN : impactUT;
            }
            catch (Exception ex)
            {
                if (Core.Landing.LandingTraceEnabled)
                    Core.Landing.TraceLanding("predictor target-aware ballistic impact unavailable: " + ex.Message);
                return double.NaN;
            }
        }

        private static void ReleaseBrakingPlanResults(IEnumerable<ReentrySimulation.Result> results)
        {
            foreach (ReentrySimulation.Result candidate in results)
            {
                LandingPredictorCapture.Discarded(candidate.CaptureSubmissionId, "queued_plan_released");
                candidate.Release();
            }
        }

        private void ReleaseQueuedBrakingPlanResults()
        {
            lock (readyBrakingPlanResults)
            {
                while (readyBrakingPlanResults.Count > 0)
                    ReleaseBrakingPlanResults(((BrakingPlanResultSet)readyBrakingPlanResults.Dequeue()).Results);
            }
        }

        private sealed class BrakingPlanJob
        {
            public readonly List<ReentrySimulation> Simulations;
            public readonly long Generation;
            public readonly List<long> CaptureIds;

            public BrakingPlanJob(List<ReentrySimulation> simulations, long generation, List<long> captureIds)
            {
                Simulations = simulations;
                Generation = generation;
                CaptureIds = captureIds;
            }
        }

        private sealed class BrakingPlanResultSet
        {
            public readonly long Generation;
            public readonly List<ReentrySimulation.Result> Results;

            public BrakingPlanResultSet(long generation, List<ReentrySimulation.Result> results)
            {
                Generation = generation;
                Results = results;
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
            // Once a target-aware candidate has been selected, retain it until
            // the next bounded planning refresh. Ordinary five-per-second
            // scalar-policy predictions are diagnostic input for that refresh;
            // they must not briefly replace the target-aware endpoint used by
            // V1's existing coast and braking steps.
            if (result != null && !double.IsNaN(result.InputForcedBrakingStartUT) &&
                newResult.InputUT - result.InputUT < BrakingPlanRefreshSeconds)
            {
                TraceNormalResultDecision("retain_target_aware_plan", result, newResult, terrainTrace);
                newResult.Release();
                return;
            }

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
            if ((MapView.MapIsEnabled || camTrajectory) && !Vessel.LandedOrSplashed && Enabled)
            {
                ReentrySimulation.Result drawnResult = Result;
                if (drawnResult != null)
                {
                    if (drawnResult.Outcome == ReentrySimulation.Outcome.LANDED)
                        GLUtils.DrawGroundMarker(drawnResult.Body, drawnResult.EndPosition.Latitude, drawnResult.EndPosition.Longitude, Color.blue,
                            MapView.MapIsEnabled, 60);

                    if (showTrajectory && drawnResult.Outcome != ReentrySimulation.Outcome.ERROR &&
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
                                    GLUtils.DrawPath(drawnResult.Body, coast.value, Color.cyan,
                                        MapView.MapIsEnabled);
                            using (Disposable<List<Vector3d>> burn = drawnResult.WorldTrajectorySegment(
                                interval, worldTrajectory, drawnResult.SimulatedBrakingStartUT,
                                drawnResult.EndUT))
                                if (burn.value.Count > 1)
                                    GLUtils.DrawPath(drawnResult.Body, burn.value, Color.red,
                                        MapView.MapIsEnabled);
                        }
                        else using (Disposable<List<Vector3d>> list = drawnResult.WorldTrajectory(
                            interval, worldTrajectory))
                        {
                            if (!MapView.MapIsEnabled && (noSkipToFreefall || Vessel.staticPressurekPa > 0))
                                list.value[0] = VesselState.CoM;
                            GLUtils.DrawPath(drawnResult.Body, list.value, Color.red, MapView.MapIsEnabled);
                        }
                    }
                }
            }
        }

        public MechJebModuleLandingPredictions(MechJebCore core) : base(core) { }
    }
}
