extern alias JetBrainsAnnotations;
using System;
using System.Collections;
using System.Collections.Generic;
using Stopwatch = System.Diagnostics.Stopwatch;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using JetBrainsAnnotations::JetBrains.Annotations;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// V2 airless landing module.  Preview remains opt-in; the controller is a
    /// separate opt-in path and never starts or changes the restored V1 controller.
    /// </summary>
    public class MechJebModuleLandingGuidanceV2 : ComputerModule
    {
        private const double RefreshInterval = 0.5;
        private const double PlanRefreshInterval = 30.0;
        private double _nextRefreshUT;
        private double _nextPlanRefreshUT;
        private double _lastAirlessPlanningMilliseconds = double.NaN;
        private long _snapshotVersion;
        private long _v1PredictionVersion;
        private ReentrySimulation.Result _lastV1Prediction;
        private ReentrySimulation.Result _lastAtmosphericPlanPrediction;
        private readonly Queue _readyAtmosphericCandidateResults = new Queue();
        private readonly Queue _readyAtmosphericEntryPlanningResults = new Queue();
        // Airless strategic planning is also worker-owned.  Its input is an
        // immutable snapshot, so a long Lambert/plane search cannot block a
        // Unity physics update or make auto-warp stutter.
        private readonly Queue _readyAirlessPlanningResults = new Queue();
        private AirlessLandingPlan _lastCompletedAirlessPlan;
        private bool _airlessPlanningRunning;
        private long _airlessPlanningGeneration;
        private ReentrySimulation.Result _atmosphericCandidateResult;
        private LandingGuidanceV2Snapshot _atmosphericCandidateSnapshot;
        private AtmosphericLandingPlan _atmosphericCandidatePlan;
        private bool _atmosphericCandidateSimulationRunning;
        private bool _atmosphericEntryPlanningRunning;
        private AtmosphericLandingPlan _lastAtmosphericEntryPlan;
        private LandingGuidanceV2Snapshot _lastAtmosphericEntryPlanningSnapshot;
        private double _lastAtmosphericEntryPlanningMilliseconds = double.NaN;
        private double _nextAtmosphericCandidateSimulationUT;
        private long _atmosphericCandidateGeneration;
        private bool _atmosphericFreshValidationRequested;
        private AtmosphericEntryPhaseManager _atmosphericPhaseManager;
        private string _lastV1Phase;
        private bool? _lastV1Burning;
        private bool? _lastWarped;
        private V2FlightPhase _flightPhase;
        private AirlessLandingPlan _activePlan;
        private Vector3d _burnTargetVelocity;
        private Vector3d _trimDeltaV;
        private Vector3d _lastAdjustedVelocity;
        private double _phaseBurnPlannedDeltaV;
        private string _phaseBurnName;
        private FiniteBurnProgress _finiteBurnProgress;
        private bool _finiteBurnTracking;
        private Vector3d _commandedV2AttitudeVector;
        // Pure phase gate shared with the controller-validation harness. It
        // decides when authority may be requested; this module performs the
        // resulting KSP commands.
        private readonly AirlessLandingPhaseManager _airlessPhaseManager = new AirlessLandingPhaseManager();
        private readonly AirlessTerminalWarpGate _terminalWarpGate = new AirlessTerminalWarpGate();
        // V2 alone owns the decision to stage during its commanded burns. The
        // generic staging module remains only the actuator after this gate.
        private readonly V2PoweredStagingGate _v2PoweredStagingGate = new V2PoweredStagingGate();
        private double _lastV2StageUT = double.NegativeInfinity;
        private string _lastV2Phase;
        private double _activeTargetLatitude;
        private double _activeTargetLongitude;
        private double _originalTargetLatitude;
        private double _originalTargetLongitude;
        private bool _hasActiveTarget;
        // The target's body-fixed coordinates are immutable through a plan.
        // This inertial reference and epoch are captured only when the user
        // starts V2 or V2 explicitly rebases/diverts locally. Fresh vessel
        // snapshots must retain them so a burn-gate validation evaluates the
        // same rotating target that authorized the original plan.
        private double _targetReferenceUT = double.NaN;
        private Vector3d _targetReferencePosition;
        private bool _hasTargetReference;
        private bool _visualRebaseDone;
        private bool _visualAssessmentCompleted;
        // Set as soon as V2 commits a finite airless deorbit burn. From this
        // point V2 must retain controlled flight through a terminal outcome;
        // it may not release an impact trajectory because a correction gate
        // failed.
        private bool _airlessDescentCommitted;
        private LandingSiteAssessment _siteAssessment;
        private string _pendingTargetEvent;
        private const double VisualAssessmentAltitude = 750.0;
        private const double VisualRebaseAccuracyLimit = 500.0;
        // Terminal hoverslam ignition is a burn *deadline*, not the point at
        // which V2 may first begin acquiring braking attitude.  Leave coast
        // with enough 1x time to settle the physical thrust vector.
        private const double TerminalBrakingAlignmentLeadSeconds = 15.0;
        // V2 owns these engine settings only while its final finite-burn
        // remapping is active.  They are restored before every control exit.
        private readonly Dictionary<ModuleEngines, float> _thrustLimitsBeforeV2FineControl =
            new Dictionary<ModuleEngines, float>();
        // Trace the physical fine-thrust mapping separately from MechJeb's
        // main-throttle safety cap.  A diagnostic record can then show that a
        // low commanded throttle was deliberately remapped through engine
        // thrustPercentage, rather than being an unexplained loss of thrust.
        private double _v2FineEngineRelativeLimit = 1;
        private double _v2FineEngineExpectedAcceleration = double.NaN;
        private int _v2FineEngineCount;

        public enum V2FlightPhase { Idle, Preflight, WarpToStrategic, AlignPlane, PlaneAlignment, AlignStrategicBurn, StrategicBurn, AlignTrim, BoundedTrim, Coast, WarpToAtmosphericEntry, AlignAtmosphericEntryBurn, AtmosphericEntryBurn, AtmosphericEntry, BrakingApproach, VisualAssessment, TerminalDivert, VelocityNull, Complete, Rejected }

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool PreviewEnabled;

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool StructuredTraceEnabled;

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool V2AutoWarp = true;

        public LandingGuidanceV2Preflight Preflight { get; private set; }

        public bool IsPreviewOnly => _flightPhase == V2FlightPhase.Idle || _flightPhase == V2FlightPhase.Rejected || _flightPhase == V2FlightPhase.Complete;
        public bool ControllerActive => _flightPhase == V2FlightPhase.Preflight || _flightPhase == V2FlightPhase.WarpToStrategic || _flightPhase == V2FlightPhase.AlignPlane || _flightPhase == V2FlightPhase.PlaneAlignment || _flightPhase == V2FlightPhase.AlignStrategicBurn || _flightPhase == V2FlightPhase.StrategicBurn || _flightPhase == V2FlightPhase.AlignTrim || _flightPhase == V2FlightPhase.BoundedTrim || _flightPhase == V2FlightPhase.Coast || _flightPhase == V2FlightPhase.WarpToAtmosphericEntry || _flightPhase == V2FlightPhase.AlignAtmosphericEntryBurn || _flightPhase == V2FlightPhase.AtmosphericEntryBurn || _flightPhase == V2FlightPhase.AtmosphericEntry || _flightPhase == V2FlightPhase.BrakingApproach || _flightPhase == V2FlightPhase.VisualAssessment || _flightPhase == V2FlightPhase.TerminalDivert || _flightPhase == V2FlightPhase.VelocityNull;
        public V2FlightPhase FlightPhase => _flightPhase;
        public string ControllerStatus { get; private set; } = "Idle";
        public bool HasV2ActiveTarget => _hasActiveTarget;
        public double V2ActiveTargetLatitude => _hasActiveTarget ? _activeTargetLatitude : (double)Core.Target.targetLatitude;
        public double V2ActiveTargetLongitude => _hasActiveTarget ? _activeTargetLongitude : (double)Core.Target.targetLongitude;
        public double V2OriginalTargetLatitude => _hasActiveTarget ? _originalTargetLatitude : (double)Core.Target.targetLatitude;
        public double V2OriginalTargetLongitude => _hasActiveTarget ? _originalTargetLongitude : (double)Core.Target.targetLongitude;
        public LandingGuidanceV2TargetState TargetState
        {
            get
            {
                LandingGuidanceV2Estimate estimate = Preflight?.Estimate;
                double predictedLatitude = double.NaN;
                double predictedLongitude = double.NaN;
                if (estimate != null && estimate.HasImpact && MainBody != null)
                    MainBody.GetLatLngAltAtUT(estimate.ImpactUT, estimate.ImpactPosition, out predictedLatitude, out predictedLongitude, out _);
                else
                {
                    // During V2 atmospheric planning the blue marker must show
                    // V2's burn-correlated simulation, never a legacy current-
                    // orbit prediction that has not had the proposed burn.
                    ReentrySimulation.Result atmospheric = _atmosphericCandidateResult ??
                        Core.GetComputerModule<MechJebModuleLandingPredictions>()?.Result;
                    if (atmospheric != null && atmospheric.Body == MainBody &&
                        atmospheric.Outcome == ReentrySimulation.Outcome.LANDED)
                    {
                        predictedLatitude = atmospheric.EndPosition.Latitude;
                        predictedLongitude = atmospheric.EndPosition.Longitude;
                    }
                }
                return new LandingGuidanceV2TargetState(V2OriginalTargetLatitude, V2OriginalTargetLongitude,
                    V2ActiveTargetLatitude, V2ActiveTargetLongitude, predictedLatitude, predictedLongitude,
                    estimate?.SnapshotVersion ?? -1, _visualRebaseDone);
            }
        }
        public string SiteAssessmentStatus => _siteAssessment == null ? "Waiting for the local visual-assessment gate." : _siteAssessment.Detail;

        public MechJebModuleLandingGuidanceV2(MechJebCore core) : base(core)
        {
            Enabled = true;
        }

        public override void OnStart(PartModule.StartState state)
        {
            if (state != PartModule.StartState.None && state != PartModule.StartState.Editor)
                Core.AddToPostDrawQueue(DrawV2MapMarkers);
        }

        public override void OnFixedUpdate()
        {
            if (ControllerActive)
                TickController();
            if (PreviewEnabled && HighLogic.LoadedSceneIsFlight &&
                (Core.Target.PositionTargetExists || ControllerActive && _hasActiveTarget))
                RefreshPreflight();
        }

        public bool StartLanding()
        {
            if (!HighLogic.LoadedSceneIsFlight || !Core.Target.PositionTargetExists)
                return RejectController("Select a landing target before starting V2.");
            SetActiveTarget(Core.Target.targetLatitude, Core.Target.targetLongitude, false);
            _airlessPlanningGeneration++;
            _airlessPlanningRunning = false;
            _lastCompletedAirlessPlan = null;
            _originalTargetLatitude = Core.Target.targetLatitude;
            _originalTargetLongitude = Core.Target.targetLongitude;
            _visualRebaseDone = false;
            _visualAssessmentCompleted = false;
            _airlessDescentCommitted = false;
            _terminalWarpGate.Reset();
            _v2PoweredStagingGate.Reset();
            _lastV2StageUT = double.NegativeInfinity;
            _atmosphericPhaseManager = null;
            _atmosphericFreshValidationRequested = false;
            _siteAssessment = null;
            if (MainBody.atmosphere)
            {
                // V2 uses the existing re-entry simulator only as an estimator;
                // it never starts the V1 landing controller.  The phase manager
                // waits for a fresh, corridor-valid estimate before it requests
                // attitude or throttle.
                Core.GetComputerModule<MechJebModuleLandingPredictions>()?.Users.Add(this);
                RefreshPreflight(true);
                TransitionTo(V2FlightPhase.Preflight, "V2 is acquiring a fresh atmospheric entry-corridor estimate.");
                return true;
            }
            RefreshPreflight(true);
            if (Preflight?.AirlessPlan == null || Preflight.AirlessPlan.State != AirlessLandingPlanState.Candidate)
            {
                TransitionTo(V2FlightPhase.Preflight,
                    "V2 accepted the selected target and is waiting for a feasible airless strategic-deorbit plan.");
                return true;
            }
            _activePlan = Preflight.AirlessPlan;
            if (!StartAirlessPhaseManager()) return false;
            Core.Thrust.Users.Add(this); Core.Attitude.Users.Add(this);
            Core.Hoverslam.Users.Add(this);
            TransitionTo(V2FlightPhase.WarpToStrategic, "V2 plan accepted; moving to the strategic-deorbit burn gate.");
            return true;
        }

        // Retained for save/UI compatibility while the visible V2 action is
        // now body-neutral.
        public bool StartAirlessLanding() => StartLanding();

        public void AbortAirlessLanding()
        {
            ReleaseV2Control(); TransitionTo(V2FlightPhase.Idle, "V2 landing aborted.");
        }

        private bool StartAirlessPhaseManager()
        {
            LandingGuidanceV2Snapshot snapshot = CaptureSnapshot();
            double maximumAcceleration = snapshot?.MaximumAcceleration ?? 0;
            double lead = maximumAcceleration > 0
                ? _activePlan.StrategicDeorbitDeltaVMagnitude / maximumAcceleration * 0.5 + 0.10
                : 0;
            AirlessLandingPhaseDecision decision = _airlessPhaseManager.Start(_activePlan, lead);
            if (decision.Directive != AirlessLandingPhaseDirective.Reject) return true;
            RejectController(decision.Reason);
            return false;
        }

        private bool StartAtmosphericPhaseManager(AtmosphericLandingPlan plan)
        {
            LandingGuidanceV2Snapshot snapshot = CaptureSnapshot();
            double maximumAcceleration = snapshot?.MaximumAcceleration ?? 0;
            double lead = maximumAcceleration > 0
                ? plan.StrategicEntryDeltaV.magnitude / maximumAcceleration * 0.5 + 0.10
                : 0;
            _atmosphericPhaseManager = new AtmosphericEntryPhaseManager(lead);
            _atmosphericFreshValidationRequested = false;
            AtmosphericEntryPhaseDecision decision = _atmosphericPhaseManager.Start(plan);
            if (decision.Directive != AtmosphericEntryDirective.Reject) return true;
            RejectController(decision.Reason);
            return false;
        }

        // A candidate is tied to a single immutable orbit snapshot. At the
        // warp-exit and post-burn boundaries invalidate the old simulation,
        // queue a new worker result, and keep V2's own attitude/thrust users
        // until that result is accepted or explicitly rejected.
        private void BeginAtmosphericFreshValidation()
        {
            if (_atmosphericFreshValidationRequested) return;
            _atmosphericFreshValidationRequested = true;
            ClearAtmosphericCandidateResult();
            Core.GetComputerModule<MechJebModuleLandingPredictions>()?.Users.Add(this);
            RefreshPreflight(true, true);
        }

        private bool TryAcceptAtmosphericFreshValidation(bool postBurn)
        {
            BeginAtmosphericFreshValidation();
            RefreshPreflight(false);
            AtmosphericLandingPlan plan = Preflight?.AtmosphericPlan;
            if (plan == null || plan.State == AtmosphericLandingPlanState.WaitingForEstimate)
            {
                ControllerStatus = "V2 is holding at 1x while its independent atmospheric simulation validates the current trajectory.";
                return false;
            }
            if (_atmosphericPhaseManager == null)
            {
                RejectController("V2 atmospheric validation completed after its phase authority was lost.");
                return false;
            }
            AtmosphericEntryPhaseDecision decision = postBurn
                ? _atmosphericPhaseManager.AcceptPostBurnValidation(plan)
                : _atmosphericPhaseManager.AcceptFreshBurnValidation(plan, Preflight.Snapshot.UT);
            if (decision.Directive == AtmosphericEntryDirective.Reject)
            {
                RejectController(decision.Reason);
                return false;
            }
            _atmosphericCandidatePlan = _atmosphericPhaseManager.Plan;
            _atmosphericFreshValidationRequested = false;
            return true;
        }

        private bool RejectController(string reason)
        {
            // A rejected start has not acquired V2 authority and must not
            // change a player's manual warp. An active V2 run releases its
            // own warp, thrust and attitude authority together.
            if (ControllerActive && _airlessDescentCommitted && Vessel != null && !Vessel.LandedOrSplashed)
            {
                EnterAirlessTerminalContingency(reason);
                return false;
            }
            if (ControllerActive) ReleaseV2Control();
            else ClearAtmosphericCandidateResult();
            TransitionTo(V2FlightPhase.Rejected, reason);
            return false;
        }

        private void TickController()
        {
            if (Vessel == null || Vessel.LandedOrSplashed) { ReleaseV2Control(); TransitionTo(V2FlightPhase.Complete, "V2 landing completed: vessel is landed or splashed."); return; }
            if (Core.Landing != null && Core.Landing.Enabled)
            {
                // V1 is the selected controller. Do not change its warp or
                // any other command while V2 removes only its own users.
                ReleaseV2Control(false);
                TransitionTo(V2FlightPhase.Rejected, "V1 Landing Guidance was engaged; V2 relinquished control.");
                return;
            }
            if (!MainBody.atmosphere && ControllerActive && _flightPhase != V2FlightPhase.Preflight &&
                AirlessAuthorityGate.Decide(VesselState.MaxThrustAcceleration, Vessel.graviticAcceleration.magnitude,
                    out string authorityReason) == AirlessAuthorityAction.Abort)
            {
                // This is an irreversible live-vessel failure, rather than a
                // normal preflight rejection. Do not leave a stale plan able
                // to command further warp or enter a terminal loop without
                // propulsion. The trace records the explicit phase event.
                ReleaseV2Control();
                TransitionTo(V2FlightPhase.Rejected, authorityReason);
                return;
            }
            UpdateFiniteBurnProgress();
            switch (_flightPhase)
            {
                case V2FlightPhase.Preflight:
                    // Start and command gates force an immediate plan. While
                    // safely waiting, the normal planner cadence is the defined
                    // replan event; do not consume a full strategic search every
                    // physics frame.
                    RefreshPreflight(false);
                    if (MainBody.atmosphere)
                    {
                        if (Preflight?.AtmosphericPlan?.State == AtmosphericLandingPlanState.Candidate)
                        {
                            _atmosphericCandidatePlan = Preflight.AtmosphericPlan;
                            if (!StartAtmosphericPhaseManager(_atmosphericCandidatePlan)) break;
                            Core.Thrust.Users.Add(this);
                            Core.Attitude.Users.Add(this);
                            if (_atmosphericPhaseManager.Phase == AtmosphericEntryPhase.Entry)
                                TransitionTo(V2FlightPhase.AtmosphericEntry,
                                    "V2 validated the current atmospheric entry trajectory; beginning the independent entry profile.");
                            else
                                TransitionTo(V2FlightPhase.WarpToAtmosphericEntry,
                                    "V2 validated its strategic atmospheric entry burn; moving to its staged burn gate.");
                        }
                        else if (Preflight?.AtmosphericPlan?.State == AtmosphericLandingPlanState.Rejected)
                            RejectController(Preflight.AtmosphericPlan.Reason);
                        else
                            ControllerStatus = "V2 is independently simulating the atmospheric entry candidate before requesting any vessel command.";
                    }
                    else if (Preflight?.AirlessPlan?.State == AirlessLandingPlanState.Candidate)
                    {
                        _activePlan = Preflight.AirlessPlan;
                        if (!StartAirlessPhaseManager()) break;
                        Core.Thrust.Users.Add(this);
                        Core.Attitude.Users.Add(this);
                        TransitionTo(V2FlightPhase.WarpToStrategic,
                            "V2 airless plan is now feasible; moving to the strategic-deorbit burn gate.");
                    }
                    else
                    {
                        ControllerStatus = "V2 is holding in airless preflight and will replan before requesting any vessel command.";
                    }
                    break;
                case V2FlightPhase.WarpToStrategic:
                    Core.Thrust.ThrustOff();
                    bool leavingInitialWarp = _airlessPhaseManager.Phase == AirlessLandingPhaseManagerPhase.InitialWarpToPlaneAlignment ||
                        _airlessPhaseManager.Phase == AirlessLandingPhaseManagerPhase.InitialWarpToStrategicBurn;
                    // A burn attitude is acquired before V2 is permitted to
                    // enter rails.  The phase manager returns no warp command
                    // until this observation is inside its authority gate.
                    bool preparingPlaneWarp = _airlessPhaseManager.Phase == AirlessLandingPhaseManagerPhase.PreparePlaneAlignmentWarp;
                    bool preparingStrategicWarp = _airlessPhaseManager.Phase == AirlessLandingPhaseManagerPhase.PrepareStrategicWarp;
                    if (preparingPlaneWarp || preparingStrategicWarp)
                    {
                        Vector3d preWarpBurn = preparingPlaneWarp
                            ? _activePlan.PlaneAlignmentDeltaV
                            : _activePlan.StrategicDeorbitDeltaV;
                        Core.Attitude.attitudeTo(preWarpBurn, AttitudeReference.INERTIAL_COT, this);
                        _commandedV2AttitudeVector = preWarpBurn;
                    }
                    Vector3d scheduledWarpBurn = preparingPlaneWarp ? _activePlan.PlaneAlignmentDeltaV : _activePlan.StrategicDeorbitDeltaV;
                    AirlessLandingPhaseDecision warpDecision = _airlessPhaseManager.Tick(VesselState.Time, V2AutoWarp,
                        BurnAlignmentError(scheduledWarpBurn), double.NaN);
                    if (warpDecision.Directive == AirlessLandingPhaseDirective.Reject)
                    {
                        RejectController(warpDecision.Reason);
                        break;
                    }
                    if (warpDecision.Directive == AirlessLandingPhaseDirective.RequestWarp)
                    {
                        Core.Warp.WarpToUT(warpDecision.WarpUT);
                        break;
                    }
                    if (warpDecision.Directive == AirlessLandingPhaseDirective.RequestInitialWarp)
                    {
                        Core.Warp.WarpToUT(warpDecision.WarpUT);
                        ControllerStatus = "V2 is coarse-warping to the 10-minute pre-burn alignment gate.";
                        break;
                    }
                    if (warpDecision.Directive == AirlessLandingPhaseDirective.RequestAttitude)
                    {
                        if (leavingInitialWarp)
                        {
                            Core.Warp.MinimumWarp(true);
                            RefreshPreflight(false, true);
                        }
                        ControllerStatus = "V2 is holding at 1x until the next finite burn attitude is confirmed before warp.";
                        break;
                    }
                    if (warpDecision.Directive == AirlessLandingPhaseDirective.WarpAuthorized)
                    {
                        ControllerStatus = "V2 confirmed the next burn attitude at 1x; auto-warp is now authorized.";
                        break;
                    }
                    // The scheduler has exited rails. The phase manager marks
                    // this boundary so a new ignition snapshot is mandatory.
                    Core.Warp.MinimumWarp(true);
                    bool needsPlaneAlignment = _airlessPhaseManager.Phase == AirlessLandingPhaseManagerPhase.AlignPlaneAlignment;
                    _burnTargetVelocity = VesselState.OrbitalVelocity +
                        (needsPlaneAlignment ? _activePlan.PlaneAlignmentDeltaV : _activePlan.StrategicDeorbitDeltaV);
                    TransitionTo(needsPlaneAlignment ? V2FlightPhase.AlignPlane : V2FlightPhase.AlignStrategicBurn,
                        needsPlaneAlignment ? "Settled at 1x; aligning for the scheduled V2 plane-alignment burn." :
                        "Settled at 1x; aligning for the scheduled V2 strategic deorbit burn.");
                    break;
                case V2FlightPhase.AlignPlane:
                    Core.Thrust.ThrustOff();
                    Core.Attitude.attitudeTo(_activePlan.PlaneAlignmentDeltaV, AttitudeReference.INERTIAL_COT, this);
                    _commandedV2AttitudeVector = _activePlan.PlaneAlignmentDeltaV;
                    AirlessLandingPhaseDecision planeAlignDecision = _airlessPhaseManager.Tick(VesselState.Time, false,
                        BurnAlignmentError(_activePlan.PlaneAlignmentDeltaV), double.NaN);
                    if (planeAlignDecision.Directive == AirlessLandingPhaseDirective.RequireFreshPlaneValidation)
                    {
                        LandingGuidanceV2Snapshot planeSnapshot = CaptureSnapshot();
                        bool valid = AirlessLandingPlanner.TryValidateCommittedPlaneAlignmentBurn(planeSnapshot,
                            _activePlan, out string validationReason);
                        planeAlignDecision = _airlessPhaseManager.AcceptPlaneAlignmentValidation(
                            planeSnapshot.Version, planeSnapshot.UT, valid, validationReason);
                        if (planeAlignDecision.Directive == AirlessLandingPhaseDirective.Reject)
                        {
                            RejectController("V2 plan failed fresh validation at the plane-alignment ignition gate: " +
                                planeAlignDecision.Reason);
                            break;
                        }
                        SetValidatedAirlessPreflight(planeSnapshot, _activePlan);
                        planeAlignDecision = _airlessPhaseManager.Tick(VesselState.Time, false,
                            BurnAlignmentError(_activePlan.PlaneAlignmentDeltaV), double.NaN);
                    }
                    if (planeAlignDecision.Directive == AirlessLandingPhaseDirective.Reject)
                    {
                        RejectController(planeAlignDecision.Reason);
                        break;
                    }
                    if (planeAlignDecision.Directive == AirlessLandingPhaseDirective.BeginFiniteBurn)
                    {
                        BeginFiniteBurn("plane_alignment", _activePlan.PlaneAlignmentDeltaVMagnitude);
                        TransitionTo(V2FlightPhase.PlaneAlignment, "Executing the finite V2 plane-alignment burn.");
                    }
                    break;
                case V2FlightPhase.PlaneAlignment:
                    Core.Attitude.attitudeTo(_activePlan.PlaneAlignmentDeltaV, AttitudeReference.INERTIAL_COT, this);
                    _commandedV2AttitudeVector = _activePlan.PlaneAlignmentDeltaV;
                    double remainingPlaneDv = RemainingFiniteBurnDeltaV;
                    AirlessLandingPhaseDecision planeBurnDecision = _airlessPhaseManager.Tick(VesselState.Time, false,
                        BurnAlignmentError(_activePlan.PlaneAlignmentDeltaV), remainingPlaneDv);
                    if (planeBurnDecision.Directive == AirlessLandingPhaseDirective.RequestFiniteBurnThrottle)
                    {
                        CommandFiniteBurnThrottle(remainingPlaneDv);
                        break;
                    }
                    if (planeBurnDecision.Directive == AirlessLandingPhaseDirective.RequestAttitude)
                    {
                        Core.Thrust.ThrustOff();
                        break;
                    }
                    if (planeBurnDecision.Directive != AirlessLandingPhaseDirective.FiniteBurnComplete)
                    {
                        RejectController(planeBurnDecision.Reason ?? "V2 plane-alignment phase did not retain finite-burn authority.");
                        break;
                    }
                    FinishFiniteBurn();
                    RefreshPreflight(true);
                    if (Preflight?.AirlessPlan == null || Preflight.AirlessPlan.State != AirlessLandingPlanState.Candidate)
                    {
                        RejectController("V2 plan failed fresh validation after plane alignment.");
                        break;
                    }
                    _activePlan = Preflight.AirlessPlan;
                    AirlessLandingPhaseDecision replanDecision = _airlessPhaseManager.AdoptStrategicReplan(_activePlan);
                    if (replanDecision.Directive == AirlessLandingPhaseDirective.Reject)
                    {
                        RejectController(replanDecision.Reason);
                        break;
                    }
                    TransitionTo(V2FlightPhase.WarpToStrategic, "Plane alignment complete; V2 is revalidating the strategic-deorbit gate.");
                    break;
                case V2FlightPhase.AlignStrategicBurn:
                    Core.Thrust.ThrustOff();
                    Core.Attitude.attitudeTo(_activePlan.StrategicDeorbitDeltaV, AttitudeReference.INERTIAL_COT, this);
                    _commandedV2AttitudeVector = _activePlan.StrategicDeorbitDeltaV;
                    AirlessLandingPhaseDecision strategicAlignDecision = _airlessPhaseManager.Tick(VesselState.Time, false,
                        BurnAlignmentError(_activePlan.StrategicDeorbitDeltaV), double.NaN);
                    if (strategicAlignDecision.Directive == AirlessLandingPhaseDirective.RequireFreshStrategicValidation)
                    {
                        LandingGuidanceV2Snapshot burnSnapshot = CaptureSnapshot();
                        bool valid = AirlessLandingPlanner.TryValidateCommittedStrategicBurn(burnSnapshot, _activePlan,
                            out AirlessLandingPlan validatedPlan, out string validationReason);
                        AirlessLandingPhaseDecision validationDecision = _airlessPhaseManager.AcceptStrategicValidation(
                            burnSnapshot.Version, burnSnapshot.UT, valid, validationReason);
                        if (validationDecision.Directive == AirlessLandingPhaseDirective.Reject)
                        {
                            RejectController("V2 plan failed fresh validation at the strategic ignition gate: " + validationDecision.Reason);
                            break;
                        }
                        _activePlan = validatedPlan;
                        SetValidatedAirlessPreflight(burnSnapshot, validatedPlan);
                        _burnTargetVelocity = VesselState.OrbitalVelocity + _activePlan.StrategicDeorbitDeltaV;
                        strategicAlignDecision = _airlessPhaseManager.Tick(VesselState.Time, false,
                            BurnAlignmentError(_activePlan.StrategicDeorbitDeltaV), double.NaN);
                    }
                    if (strategicAlignDecision.Directive == AirlessLandingPhaseDirective.Reject)
                    {
                        RejectController(strategicAlignDecision.Reason);
                        break;
                    }
                    if (strategicAlignDecision.Directive == AirlessLandingPhaseDirective.BeginFiniteBurn)
                    {
                        BeginFiniteBurn("strategic_deorbit", _activePlan.StrategicDeorbitDeltaVMagnitude);
                        _airlessDescentCommitted = true;
                        TransitionTo(V2FlightPhase.StrategicBurn, "Executing V2 strategic deorbit burn.");
                    }
                    break;
                case V2FlightPhase.StrategicBurn:
                    Core.Attitude.attitudeTo(_activePlan.StrategicDeorbitDeltaV, AttitudeReference.INERTIAL_COT, this);
                    _commandedV2AttitudeVector = _activePlan.StrategicDeorbitDeltaV;
                    double remainingDv = RemainingFiniteBurnDeltaV;
                    AirlessLandingPhaseDecision strategicBurnDecision = _airlessPhaseManager.Tick(VesselState.Time, false,
                        BurnAlignmentError(_activePlan.StrategicDeorbitDeltaV), remainingDv);
                    if (strategicBurnDecision.Directive == AirlessLandingPhaseDirective.RequestFiniteBurnThrottle)
                    {
                        CommandFiniteBurnThrottle(remainingDv);
                        break;
                    }
                    if (strategicBurnDecision.Directive == AirlessLandingPhaseDirective.RequestAttitude)
                    {
                        Core.Thrust.ThrustOff();
                        break;
                    }
                    if (strategicBurnDecision.Directive != AirlessLandingPhaseDirective.FiniteBurnComplete)
                    {
                        RejectController(strategicBurnDecision.Reason ?? "V2 strategic phase did not retain finite-burn authority.");
                        break;
                    }
                    FinishFiniteBurn();
                    RefreshPreflight(false, true);
                    LandingGuidanceV2Snapshot trimSnapshot = CaptureSnapshot();
                    AirlessPostBurnDecision postBurn = AirlessLandingPlanner.DecidePostBurn(trimSnapshot, _activePlan, true);
                    if (postBurn.Action == AirlessPostBurnAction.RecoveryTrim)
                    {
                        _trimDeltaV = postBurn.Correction;
                        _burnTargetVelocity = VesselState.OrbitalVelocity + _trimDeltaV;
                        TransitionTo(V2FlightPhase.AlignTrim,
                            postBurn.Reason + " Budget " + postBurn.RecoveryBudget.ToString("F1", CultureInfo.InvariantCulture) + " m/s.");
                    }
                    else if (postBurn.Action == AirlessPostBurnAction.Coast)
                        TransitionTo(V2FlightPhase.Coast, postBurn.Reason);
                    else
                        BeginAirlessRecoveryReplan(postBurn.Reason);
                    break;
                case V2FlightPhase.AlignTrim:
                    Core.Thrust.ThrustOff();
                    Core.Attitude.attitudeTo(_trimDeltaV, AttitudeReference.INERTIAL_COT, this);
                    _commandedV2AttitudeVector = _trimDeltaV;
                    if (BurnAlignmentReady(_trimDeltaV))
                    {
                        BeginFiniteBurn("bounded_trim", _trimDeltaV.magnitude);
                        TransitionTo(V2FlightPhase.BoundedTrim, "Executing one bounded V2 trim burn.");
                    }
                    break;
                case V2FlightPhase.BoundedTrim:
                    Core.Attitude.attitudeTo(_trimDeltaV, AttitudeReference.INERTIAL_COT, this);
                    _commandedV2AttitudeVector = _trimDeltaV;
                    if (!BurnAlignmentReady(_trimDeltaV))
                    {
                        Core.Thrust.ThrustOff();
                        TransitionTo(V2FlightPhase.AlignTrim,
                            "V2 paused the bounded trim because attitude authority left the finite-burn gate.");
                        break;
                    }
                    if (RemainingFiniteBurnDeltaV <= AirlessLandingPhaseManager.BurnCompleteDeltaV)
                    {
                        FinishFiniteBurn();
                        RefreshPreflight();
                        LandingGuidanceV2Snapshot postTrimSnapshot = CaptureSnapshot();
                        AirlessPostBurnDecision postTrim = AirlessLandingPlanner.DecidePostBurn(postTrimSnapshot, _activePlan, false);
                        if (postTrim.Action == AirlessPostBurnAction.Coast)
                            TransitionTo(V2FlightPhase.Coast, "Bounded V2 trim complete. " + postTrim.Reason);
                        else
                            BeginAirlessRecoveryReplan(postTrim.Reason);
                    }
                    else CommandFiniteBurnThrottle(RemainingFiniteBurnDeltaV);
                    break;
                case V2FlightPhase.Coast:
                    Core.Thrust.ThrustOff();
                    // A coasting airless descent remains a controlled phase.
                    // Check a current immutable endpoint at the normal bounded
                    // refresh cadence; do not run a synchronous estimator on
                    // every physics tick while waiting for hoverslam.
                    RefreshPreflight(false);
                    bool terminalIgnitionAvailable = !double.IsNaN(Core.Hoverslam.IgnitionUT) &&
                        !double.IsInfinity(Core.Hoverslam.IgnitionUT);
                    LandingGuidanceV2Snapshot coastSnapshot = Preflight?.Snapshot;
                    double terminalBrakingDeltaV = Preflight?.BrakingDeltaVLowerBound ?? double.NaN;
                    AirlessCoastSafetyAction coastSafety = AirlessCoastSafetyGate.Decide(coastSnapshot,
                        Preflight?.Estimate, _activePlan?.CorridorLimit ?? double.NaN, terminalBrakingDeltaV,
                        terminalIgnitionAvailable);
                    if (coastSafety == AirlessCoastSafetyAction.Replan)
                    {
                        BeginAirlessRecoveryReplan("V2 coast endpoint left the current target corridor; acquiring a fresh complete airless plan.");
                        break;
                    }
                    if (coastSafety == AirlessCoastSafetyAction.EmergencyBrake)
                    {
                        EnterAirlessTerminalContingency("the terminal ignition solution was unavailable or the target corridor was lost inside the calculated braking lead.");
                        break;
                    }
                    if (!terminalIgnitionAvailable)
                    {
                        Core.Attitude.attitudeTo(-VesselState.SurfaceVelocity, AttitudeReference.INERTIAL_COT, this);
                        ControllerStatus = "V2 is holding controlled coast while the current terminal-braking solution is rebuilt.";
                        break;
                    }
                    Core.Attitude.attitudeTo(Core.Hoverslam.IgnitionAttitude, AttitudeReference.INERTIAL_COT, this);
                    _commandedV2AttitudeVector = Core.Hoverslam.IgnitionAttitude;
                    if (!_terminalWarpGate.ObserveAttitude(VesselState.Time,
                        BurnAlignmentError(Core.Hoverslam.IgnitionAttitude)))
                    {
                        Core.Thrust.ThrustOff();
                        ControllerStatus = "V2 is holding at 1x until terminal-braking attitude remains inside the authority gate.";
                        break;
                    }
                    if (V2AutoWarp && VesselState.Time < Core.Hoverslam.IgnitionUT - 10.0)
                    {
                        // The target and impact state must be evaluated at the
                        // instant rails warp is requested. The post-trim
                        // estimate is not valid authority for a later warp.
                        RefreshPreflight(false, true);
                        if (_activePlan == null || !AirlessTerminalWarpGate.EndpointIsCurrentAndWithinCorridor(
                            Preflight?.Estimate, _activePlan.CorridorLimit))
                        {
                            _terminalWarpGate.Reset();
                            BeginAirlessRecoveryReplan("V2 denied terminal auto-warp because the fresh impact endpoint left the target corridor.");
                            break;
                        }
                        Core.Warp.WarpToUT(Core.Hoverslam.IgnitionUT - 10.0);
                    }
                    else if (Core.Hoverslam.IgnitionCountdown <= TerminalBrakingAlignmentLeadSeconds)
                    {
                        Core.Warp.MinimumWarp(true);
                        // A hoverslam ignition time that was calculated before
                        // warp cannot authorize terminal control after warp has
                        // ended. Rebuild the immutable snapshot and require a
                        // current impact estimate before entering braking.
                        RefreshPreflight(false, true);
                        if (Preflight?.Estimate == null || !Preflight.Estimate.HasImpact)
                        {
                            RejectController("V2 terminal braking lost its fresh impact trajectory after warp exit.");
                            break;
                        }
                        _lastAdjustedVelocity = new Vector3d(double.NaN, double.NaN, double.NaN);
                        TransitionTo(V2FlightPhase.BrakingApproach,
                            "V2 left coast before terminal ignition to align and begin controlled braking.");
                    }
                    break;
                case V2FlightPhase.WarpToAtmosphericEntry:
                    Core.Thrust.ThrustOff();
                    if (_atmosphericPhaseManager == null || _atmosphericPhaseManager.Plan == null)
                    {
                        RejectController("V2 atmospheric controller lost its committed entry plan.");
                        break;
                    }
                    AtmosphericLandingPlan atmosphericWarpPlan = _atmosphericPhaseManager.Plan;
                    bool preparingAtmosphericWarp = _atmosphericPhaseManager.Phase == AtmosphericEntryPhase.PrepareWarp ||
                        _atmosphericPhaseManager.Phase == AtmosphericEntryPhase.WarpToBurn;
                    if (preparingAtmosphericWarp)
                    {
                        Core.Attitude.attitudeTo(atmosphericWarpPlan.StrategicEntryDeltaV, AttitudeReference.INERTIAL_COT, this);
                        _commandedV2AttitudeVector = atmosphericWarpPlan.StrategicEntryDeltaV;
                    }
                    AtmosphericEntryPhaseDecision atmosphericWarpDecision = _atmosphericPhaseManager.Tick(VesselState.Time,
                        V2AutoWarp, BurnAlignmentError(atmosphericWarpPlan.StrategicEntryDeltaV), double.NaN);
                    if (atmosphericWarpDecision.Directive == AtmosphericEntryDirective.Reject)
                    {
                        RejectController(atmosphericWarpDecision.Reason);
                        break;
                    }
                    if (atmosphericWarpDecision.Directive == AtmosphericEntryDirective.RequestInitialWarp)
                    {
                        Core.Warp.WarpToUT(atmosphericWarpDecision.WarpUT);
                        ControllerStatus = "V2 is coarse-warping to the atmospheric entry-burn alignment gate.";
                        break;
                    }
                    if (atmosphericWarpDecision.Directive == AtmosphericEntryDirective.RequestAttitude)
                    {
                        Core.Warp.MinimumWarp(true);
                        ControllerStatus = "V2 is holding at 1x until the atmospheric entry-burn attitude is aligned and settled.";
                        break;
                    }
                    if (atmosphericWarpDecision.Directive == AtmosphericEntryDirective.WarpAuthorized)
                    {
                        ControllerStatus = "V2 confirmed atmospheric entry-burn attitude at 1x; auto-warp is authorized.";
                        break;
                    }
                    if (atmosphericWarpDecision.Directive == AtmosphericEntryDirective.RequestWarp)
                    {
                        Core.Warp.WarpToUT(atmosphericWarpDecision.WarpUT);
                        break;
                    }
                    if (atmosphericWarpDecision.Directive == AtmosphericEntryDirective.ExitWarpAndRequestAttitude)
                    {
                        Core.Warp.MinimumWarp(true);
                        BeginAtmosphericFreshValidation();
                        TransitionTo(V2FlightPhase.AlignAtmosphericEntryBurn,
                            "V2 left warp at 1x and is obtaining a fresh atmospheric entry-burn validation.");
                        break;
                    }
                    if (atmosphericWarpDecision.Directive == AtmosphericEntryDirective.RequirePostBurnValidation)
                    {
                        if (TryAcceptAtmosphericFreshValidation(true))
                        {
                            if (_atmosphericPhaseManager.Phase == AtmosphericEntryPhase.Entry)
                                TransitionTo(V2FlightPhase.AtmosphericEntry,
                                    "V2 independently validated the actual post-burn atmospheric trajectory.");
                            else
                                TransitionTo(V2FlightPhase.WarpToAtmosphericEntry,
                                    "V2 post-burn validation scheduled a bounded corrective atmospheric entry burn.");
                        }
                        break;
                    }
                    if (atmosphericWarpDecision.Directive == AtmosphericEntryDirective.EnterAtmosphericEntry)
                        TransitionTo(V2FlightPhase.AtmosphericEntry,
                            "V2 is executing its independently validated atmospheric entry profile.");
                    else
                        RejectController("V2 atmospheric warp gate returned an unsupported controller directive.");
                    break;
                case V2FlightPhase.AlignAtmosphericEntryBurn:
                    Core.Thrust.ThrustOff();
                    if (_atmosphericPhaseManager == null || _atmosphericPhaseManager.Plan == null)
                    {
                        RejectController("V2 atmospheric controller lost its entry-burn plan at the ignition gate.");
                        break;
                    }
                    AtmosphericLandingPlan atmosphericAlignPlan = _atmosphericPhaseManager.Plan;
                    Core.Attitude.attitudeTo(atmosphericAlignPlan.StrategicEntryDeltaV, AttitudeReference.INERTIAL_COT, this);
                    _commandedV2AttitudeVector = atmosphericAlignPlan.StrategicEntryDeltaV;
                    AtmosphericEntryPhaseDecision atmosphericAlignDecision = _atmosphericPhaseManager.Tick(VesselState.Time,
                        false, BurnAlignmentError(atmosphericAlignPlan.StrategicEntryDeltaV), double.NaN);
                    if (atmosphericAlignDecision.Directive == AtmosphericEntryDirective.RequireFreshBurnValidation)
                    {
                        if (!TryAcceptAtmosphericFreshValidation(false)) break;
                        atmosphericAlignPlan = _atmosphericPhaseManager.Plan;
                        Core.Attitude.attitudeTo(atmosphericAlignPlan.StrategicEntryDeltaV, AttitudeReference.INERTIAL_COT, this);
                        _commandedV2AttitudeVector = atmosphericAlignPlan.StrategicEntryDeltaV;
                        atmosphericAlignDecision = _atmosphericPhaseManager.Tick(VesselState.Time, false,
                            BurnAlignmentError(atmosphericAlignPlan.StrategicEntryDeltaV), double.NaN);
                    }
                    if (atmosphericAlignDecision.Directive == AtmosphericEntryDirective.Reject)
                    {
                        RejectController(atmosphericAlignDecision.Reason);
                        break;
                    }
                    if (atmosphericAlignDecision.Directive == AtmosphericEntryDirective.RequestAttitude)
                    {
                        ControllerStatus = "V2 is holding at 1x until fresh entry validation and burn attitude are both ready.";
                        break;
                    }
                    if (atmosphericAlignDecision.Directive == AtmosphericEntryDirective.BeginFiniteBurn)
                    {
                        BeginFiniteBurn("atmospheric_strategic_entry", atmosphericAlignPlan.StrategicEntryDeltaV.magnitude);
                        TransitionTo(V2FlightPhase.AtmosphericEntryBurn, "Executing the finite V2 atmospheric entry burn.");
                        break;
                    }
                    if (atmosphericAlignDecision.Directive == AtmosphericEntryDirective.EnterAtmosphericEntry)
                    {
                        TransitionTo(V2FlightPhase.AtmosphericEntry,
                            "V2 fresh ignition validation established a direct atmospheric entry trajectory.");
                        break;
                    }
                    RejectController("V2 atmospheric ignition gate returned an unsupported controller directive.");
                    break;
                case V2FlightPhase.AtmosphericEntryBurn:
                    if (_atmosphericPhaseManager == null || _atmosphericPhaseManager.Plan == null)
                    {
                        RejectController("V2 atmospheric controller lost its entry-burn plan during execution.");
                        break;
                    }
                    AtmosphericLandingPlan atmosphericBurnPlan = _atmosphericPhaseManager.Plan;
                    Core.Attitude.attitudeTo(atmosphericBurnPlan.StrategicEntryDeltaV, AttitudeReference.INERTIAL_COT, this);
                    _commandedV2AttitudeVector = atmosphericBurnPlan.StrategicEntryDeltaV;
                    double remainingEntryDv = RemainingFiniteBurnDeltaV;
                    AtmosphericEntryPhaseDecision atmosphericBurnDecision = _atmosphericPhaseManager.Tick(VesselState.Time,
                        false, BurnAlignmentError(atmosphericBurnPlan.StrategicEntryDeltaV), remainingEntryDv);
                    if (atmosphericBurnDecision.Directive == AtmosphericEntryDirective.RequestFiniteBurnThrottle)
                    {
                        CommandFiniteBurnThrottle(remainingEntryDv);
                        break;
                    }
                    if (atmosphericBurnDecision.Directive == AtmosphericEntryDirective.RequestAttitude)
                    {
                        Core.Thrust.ThrustOff();
                        ControllerStatus = "V2 paused the atmospheric entry burn until measured thrust-vector alignment is restored.";
                        break;
                    }
                    if (atmosphericBurnDecision.Directive != AtmosphericEntryDirective.FiniteBurnComplete)
                    {
                        RejectController(atmosphericBurnDecision.Reason ?? "V2 atmospheric burn did not retain finite-burn authority.");
                        break;
                    }
                    FinishFiniteBurn();
                    BeginAtmosphericFreshValidation();
                    TransitionTo(V2FlightPhase.WarpToAtmosphericEntry,
                        "V2 entry burn complete; independently validating the actual post-burn trajectory before entry.");
                    break;
                case V2FlightPhase.AtmosphericEntry:
                    // Atmospheric flight is never warped.  Keep the vehicle
                    // retrograde to the surface-relative flow while the model
                    // based estimator is refreshed, then transfer only to the
                    // local powered braking phase when its ignition solution is
                    // current.
                    Core.Warp.MinimumWarp(true);
                    Core.Thrust.ThrustOff();
                    Core.Attitude.attitudeTo(-VesselState.SurfaceVelocity, AttitudeReference.INERTIAL_COT, this);
                    DeployV2AtmosphericParachutes();
                    RefreshPreflight(false);
                    if (Preflight?.AtmosphericPlan?.State == AtmosphericLandingPlanState.Rejected)
                    {
                        RejectController(Preflight.AtmosphericPlan.Reason);
                        break;
                    }
                    // A safely deployed chute owns the final ballistic
                    // descent on a capsule or bell-shaped vessel. Do not
                    // command a competing powered braking maneuver below it.
                    if (VesselState.ParachuteDeployed)
                    {
                        ControllerStatus = "V2 is holding the ballistic entry profile under safely deployed parachutes.";
                        break;
                    }
                    if (AtmosphericBrakingEntryRequired())
                        TransitionTo(V2FlightPhase.BrakingApproach,
                            "V2 atmospheric energy gate reached its conservative powered-braking entry.");
                    break;
                case V2FlightPhase.BrakingApproach:
                    Core.Warp.MinimumWarp(true);
                    if (MainBody.atmosphere && VesselState.ParachuteDeployed)
                    {
                        Core.Thrust.ThrustOff();
                        TransitionTo(V2FlightPhase.AtmosphericEntry,
                            "V2 returned to its ballistic parachute descent profile before powered braking.");
                        break;
                    }
                    if (!MainBody.atmosphere && VesselState.AltitudeBottom <= VisualAssessmentAltitude && !_visualAssessmentCompleted)
                    {
                        Core.Warp.MinimumWarp(true);
                        TransitionTo(V2FlightPhase.VisualAssessment, "V2 entered the local visual-assessment gate.");
                        break;
                    }
                    double downSpeed = Math.Max(0, -Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up));
                    AtmosphericBallisticBrakingCommand atmosphericBraking = MainBody.atmosphere
                        ? AtmosphericBallisticBraking.Calculate(VesselState.AltitudeBottom, downSpeed,
                            Vessel.graviticAcceleration.magnitude, VesselState.MinThrustAcceleration,
                            VesselState.MaxThrustAcceleration, Core.Thrust.ThrottleLimit)
                        : null;
                    if (atmosphericBraking != null && !atmosphericBraking.Valid)
                    {
                        Core.Thrust.ThrustOff();
                        ControllerStatus = atmosphericBraking.RejectionReason;
                        break;
                    }
                    Vector3d adjustedVelocity = atmosphericBraking == null ? TerminalVelocityError() : VesselState.SurfaceVelocity;
                    Vector3d brakingDirection = -adjustedVelocity;
                    Core.Attitude.attitudeTo(brakingDirection, AttitudeReference.INERTIAL_COT, this);
                    _commandedV2AttitudeVector = brakingDirection;
                    if (!BurnAlignmentReady(brakingDirection))
                    {
                        Core.Thrust.ThrustOff();
                        ControllerStatus = "V2 is holding braking throttle until the measured thrust vector is aligned and settled.";
                        break;
                    }
                    if (atmosphericBraking != null)
                    {
                        AirlessFineThrustCommand fine = AirlessFineThrustControl.CalculateTerminal(
                            atmosphericBraking.RequestedThrottle, VesselState.MinThrustAcceleration,
                            VesselState.MaxThrustAcceleration, availableMainThrottle: Core.Thrust.ThrottleLimit);
                        Core.Thrust.TargetThrottle = (float)Math.Min(fine.RequestedThrottle, Core.Thrust.ThrottleLimit);
                        ApplyV2FineThrustLimit(fine);
                        ControllerStatus = "V2 is holding retrograde and tracking its conservative ballistic braking envelope.";
                        break;
                    }
                    Core.Thrust.TargetThrottle = 1.0f;
                    if (!double.IsNaN(_lastAdjustedVelocity.x) && Vector3d.Angle(_lastAdjustedVelocity, adjustedVelocity) > 10.0)
                    {
                        TransitionTo(V2FlightPhase.TerminalDivert, "V2 terminal-divert guidance is tracking the active red target.");
                    }
                    _lastAdjustedVelocity = adjustedVelocity;
                    break;
                case V2FlightPhase.VisualAssessment:
                    TickVisualAssessment();
                    break;
                case V2FlightPhase.TerminalDivert:
                    TickTerminalDivert();
                    break;
                case V2FlightPhase.VelocityNull:
                    TickVelocityNull();
                    break;
            }
            ConsiderV2PoweredStaging();
        }

        private void BeginAirlessRecoveryReplan(string reason)
        {
            Core.Thrust.ThrustOff();
            Core.Warp.MinimumWarp(true);
            _terminalWarpGate.Reset();
            // Discard any candidate made before the executed burn. The next
            // Preflight tick owns an immutable post-burn snapshot and waits for
            // its worker result before it can request a second finite burn.
            _airlessPlanningGeneration++;
            _airlessPlanningRunning = false;
            _lastCompletedAirlessPlan = null;
            RefreshPreflight(true, true);
            TransitionTo(V2FlightPhase.Preflight, reason);
        }

        private void EnterAirlessTerminalContingency(string reason)
        {
            Core.Warp.MinimumWarp(true);
            Core.Thrust.Users.Add(this);
            Core.Attitude.Users.Add(this);
            _lastAdjustedVelocity = new Vector3d(double.NaN, double.NaN, double.NaN);
            // The current impact is inside the vehicle-derived braking lead.
            // Continue with controlled braking and the existing local rebase /
            // divert path instead of releasing an impact trajectory.
            TransitionTo(V2FlightPhase.BrakingApproach,
                "V2 entered controlled terminal braking after an airless contingency: " + reason);
        }

        private void TickTerminalDivert()
        {
            Vector3d target = MainBody.GetWorldSurfacePosition(V2ActiveTargetLatitude, V2ActiveTargetLongitude,
                MainBody.TerrainAltitude(V2ActiveTargetLatitude, V2ActiveTargetLongitude, true));
            double horizontalError = Vector3d.Exclude(VesselState.Up, target - VesselState.CoM).magnitude;
            if (horizontalError < 5.0 && VesselState.AltitudeBottom < 50.0)
            {
                TransitionTo(V2FlightPhase.VelocityNull, "V2 velocity-null guidance is protecting touchdown speed and clearance.");
                return;
            }
            AirlessTerminalGuidanceCommand command = TerminalCommand(target);
            if (!command.Valid)
            {
                RejectController(command.RejectionReason);
                return;
            }
            Core.Attitude.attitudeTo(command.ThrustDirection, AttitudeReference.INERTIAL_COT, this);
            _commandedV2AttitudeVector = command.ThrustDirection;
            if (!BurnAlignmentReady(command.ThrustDirection))
            {
                Core.Thrust.ThrustOff();
                ControllerStatus = "V2 terminal divert is holding throttle until the measured thrust vector is aligned and settled.";
                return;
            }
            CommandTerminalThrottle(command);
        }

        private void TickVelocityNull()
        {
            Core.Warp.MinimumWarp(true);
            AirlessTerminalGuidanceCommand command = TerminalCommand(VesselState.CoM);
            if (!command.Valid)
            {
                RejectController(command.RejectionReason);
                return;
            }
            Core.Attitude.attitudeTo(command.ThrustDirection, AttitudeReference.INERTIAL_COT, this);
            _commandedV2AttitudeVector = command.ThrustDirection;
            if (!BurnAlignmentReady(command.ThrustDirection))
            {
                Core.Thrust.ThrustOff();
                ControllerStatus = "V2 velocity-null is holding throttle until the measured thrust vector is aligned and settled.";
                return;
            }
            CommandTerminalThrottle(command);
        }

        private void TickVisualAssessment()
        {
            Core.Warp.MinimumWarp(true);
            Vector3d brakingError = TerminalVelocityError();
            Vector3d visualBrakingDirection = -brakingError;
            Core.Attitude.attitudeTo(visualBrakingDirection, AttitudeReference.INERTIAL_COT, this);
            _commandedV2AttitudeVector = visualBrakingDirection;
            if (!BurnAlignmentReady(visualBrakingDirection))
            {
                Core.Thrust.ThrustOff();
                ControllerStatus = "V2 visual assessment is holding throttle until the measured thrust vector is aligned and settled.";
                return;
            }
            Core.Thrust.TargetThrottle = 1.0f;
            RefreshPreflight(false);
            if (Preflight?.Estimate == null || !Preflight.Estimate.HasImpact)
            {
                RejectController("V2 local visual assessment has no valid impact estimate.");
                return;
            }
            _visualAssessmentCompleted = true;
            if (VisualRebaseGate.Decide(Preflight.Estimate.TargetError, VisualRebaseAccuracyLimit) ==
                VisualTargetAction.RetainOriginalAndReportFailure)
            {
                // The landing has reached the local gate with an unacceptable
                // targeting error.  Retain red as the player's requested
                // target and record the failure; a rebase here would conceal
                // the failed long-range plan.  Terminal guidance remains
                // engaged for a controlled recovery rather than releasing an
                // already committed vehicle onto an impact trajectory.
                _pendingTargetEvent = "visual_accuracy_rejected";
                _siteAssessment = AssessLocalSite(V2ActiveTargetLatitude, V2ActiveTargetLongitude);
                TransitionTo(V2FlightPhase.TerminalDivert,
                    "V2 visual target-accuracy gate failed; the original target was retained for controlled terminal recovery.");
                return;
            }

            MainBody.GetLatLngAltAtUT(Preflight.Estimate.ImpactUT, Preflight.Estimate.ImpactPosition, out double latitude, out double longitude, out _);
            SetActiveTarget(latitude, longitude, true);
            _visualRebaseDone = true;
            _pendingTargetEvent = "visual_rebase";
            _siteAssessment = AssessLocalSite(latitude, longitude);
            TransitionTo(V2FlightPhase.TerminalDivert,
                "V2 visual rebase complete; terminal-divert guidance is tracking the assessed local target.");
        }

        // V2 owns atmospheric parachute deployment. It uses each parachute's
        // configured deploy altitude and KSP's safe-state check, measured from
        // the active target terrain. This does not use or alter V1's settings.
        private void DeployV2AtmosphericParachutes()
        {
            if (VesselState?.MainBody == null || !VesselState.MainBody.atmosphere) return;
            double landingASL;
            try { landingASL = MainBody.TerrainAltitude(V2ActiveTargetLatitude, V2ActiveTargetLongitude, true); }
            catch (Exception) { return; }
            for (int i = 0; i < VesselState.Parachutes.Count; i++)
            {
                ModuleParachute parachute = VesselState.Parachutes[i];
                if (parachute.deploymentState != ModuleParachute.deploymentStates.STOWED ||
                    parachute.deploymentSafeState != ModuleParachute.deploymentSafeStates.SAFE) continue;
                if (VesselState.AltitudeASL <= landingASL + parachute.deployAltitude)
                    parachute.Deploy();
            }
        }

        private void ConsiderV2PoweredStaging()
        {
            bool poweredBurnPhase = _flightPhase == V2FlightPhase.PlaneAlignment ||
                _flightPhase == V2FlightPhase.StrategicBurn || _flightPhase == V2FlightPhase.BoundedTrim ||
                _flightPhase == V2FlightPhase.AtmosphericEntryBurn || _flightPhase == V2FlightPhase.BrakingApproach ||
                _flightPhase == V2FlightPhase.TerminalDivert || _flightPhase == V2FlightPhase.VelocityNull;
            bool nextStageUsable = false;
            if (poweredBurnPhase && Vessel != null && Vessel.currentStage > 0)
            {
                Core.StageStats.RequestUpdate();
                int nextStage = Vessel.currentStage - 1;
                nextStageUsable = Core.StageStats.VacStats.Any(s => s.KSPStage == nextStage && s.DeltaV > 1.0 &&
                    s.MaxThrust > 0 && s.DeltaTime > 0);
            }
            V2PoweredStageDecision decision = _v2PoweredStagingGate.Decide(VesselState.Time, poweredBurnPhase,
                VesselState.ParachuteDeployed, Core.Thrust.TargetThrottle, VesselState.CurrentThrustAcceleration,
                VesselState.MaxThrustAcceleration, nextStageUsable);
            if (decision.Directive != V2PoweredStageDirective.CommandStage ||
                VesselState.Time - _lastV2StageUT < 2.0) return;

            // Remove thrust before activating the next KSP stage. The next
            // physics frame supplies fresh measured acceleration to V2; no
            // old-stage thrust observation is permitted to continue a burn.
            Core.Thrust.ThrustOff();
            Core.Staging.ImmediateStage();
            _lastV2StageUT = VesselState.Time;
            _v2PoweredStagingGate.Reset();
            _pendingTargetEvent = "v2_stage_command";
            ControllerStatus = "V2 staged after a sustained measured loss of full-throttle propulsion.";
        }

        // The shared hoverslam estimate does not model atmospheric drag. V2
        // enters powered braking from current physical state instead: twice
        // the ideal stopping distance leaves a full stopping-distance margin
        // for drag variation, attitude settling, and finite engine response.
        private bool AtmosphericBrakingEntryRequired()
        {
            double gravity = Vessel.graviticAcceleration.magnitude;
            double descentSpeed = Math.Max(0, -Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up));
            return AtmosphericEnergyGate.ShouldBeginPoweredBraking(VesselState.AltitudeBottom, descentSpeed,
                gravity, VesselState.MaxThrustAcceleration);
        }

        private Vector3d TerminalVelocityError()
        {
            Vector3d target = MainBody.GetWorldSurfacePosition(V2ActiveTargetLatitude, V2ActiveTargetLongitude,
                MainBody.TerrainAltitude(V2ActiveTargetLatitude, V2ActiveTargetLongitude, true));
            return TerminalCommand(target).VelocityError;
        }

        private AirlessTerminalGuidanceCommand TerminalCommand(Vector3d target) =>
            AirlessTerminalGuidance.Calculate(target - VesselState.CoM, VesselState.SurfaceVelocity, VesselState.Up,
                VesselState.AltitudeBottom, Vessel.graviticAcceleration.magnitude, VesselState.MinThrustAcceleration,
                VesselState.MaxThrustAcceleration, Core.Hoverslam.FinalDescentSpeed, Core.Thrust.ThrottleLimit);

        private void CommandTerminalThrottle(AirlessTerminalGuidanceCommand terminalCommand)
        {
            AirlessFineThrustCommand command = AirlessFineThrustControl.CalculateTerminal(
                terminalCommand.RequestedThrottle, VesselState.MinThrustAcceleration, VesselState.MaxThrustAcceleration,
                availableMainThrottle: Core.Thrust.ThrottleLimit);
            Core.Thrust.TargetThrottle = (float)Math.Min(command.RequestedThrottle, Core.Thrust.ThrottleLimit);
            ApplyV2FineThrustLimit(command);
        }

        public bool TryAdjustV2Target(double northMeters, double eastMeters)
        {
            if (!ControllerActive || (_flightPhase != V2FlightPhase.VisualAssessment && _flightPhase != V2FlightPhase.TerminalDivert))
            {
                ControllerStatus = "V2 target movement is available only during local assessment or terminal descent.";
                return false;
            }
            double latitude = V2ActiveTargetLatitude + northMeters * 180.0 / (Math.PI * MainBody.Radius);
            double longitude = V2ActiveTargetLongitude + eastMeters * 180.0 /
                (Math.PI * MainBody.Radius * Math.Max(0.01, Math.Cos(V2ActiveTargetLatitude * Math.PI / 180.0)));
            if (!CanAcceptDivert(latitude, longitude, out string reason))
            {
                ControllerStatus = reason;
                return false;
            }
            double previousLatitude = _activeTargetLatitude;
            double previousLongitude = _activeTargetLongitude;
            SetActiveTarget(latitude, longitude, true);
            _siteAssessment = AssessLocalSite(latitude, longitude);
            if (!_siteAssessment.Accepted)
            {
                SetActiveTarget(previousLatitude, previousLongitude, false);
                ControllerStatus = "V2 target movement rejected: " + _siteAssessment.Detail;
                return false;
            }
            _pendingTargetEvent = "divert_accepted";
            ControllerStatus = "V2 local divert accepted; terminal guidance is tracking the updated target.";
            RefreshPreflight();
            return true;
        }

        private bool CanAcceptDivert(double latitude, double longitude, out string reason)
        {
            Vector3d current = MainBody.GetWorldSurfacePosition(V2ActiveTargetLatitude, V2ActiveTargetLongitude, 0);
            Vector3d proposed = MainBody.GetWorldSurfacePosition(latitude, longitude, 0);
            double distance = Vector3d.Distance(current, proposed);
            double verticalSpeed = -Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up);
            double timeToGround = Math.Max(3.0, VesselState.AltitudeBottom / Math.Max(0.5, verticalSpeed));
            double divertCost = 4.0 * distance / timeToGround;
            Core.StageStats.RequestUpdate();
            double remainingDeltaV = Core.StageStats.VacStats.Sum(s => s.DeltaV);
            // The player-facing local adjustment is paid from the plan's
            // protected reserve on both body types. A fixed atmospheric value
            // would bypass the uncertainty-based entry budget.
            double reserve = _activePlan != null
                ? _activePlan.TerminalDivertReserve
                : Preflight?.AtmosphericPlan?.TerminalReserve ?? 20.0;
            if (remainingDeltaV < reserve + divertCost)
            {
                reason = "V2 target movement rejected: it would consume the protected terminal-divert reserve.";
                return false;
            }
            reason = null;
            return true;
        }

        private LandingSiteAssessment AssessLocalSite(double latitude, double longitude)
        {
            double sampleRadius = Math.Max(10.0, Math.Min(50.0, VesselState.AltitudeBottom * 0.1));
            double slope = MainBody.GetPQSSlopeDegrees(latitude, longitude, sampleRadius);
            double epsilon = sampleRadius * 180.0 / (Math.PI * MainBody.Radius);
            double center = MainBody.TerrainAltitude(latitude, longitude, true);
            double roughness = new[]
            {
                Math.Abs(MainBody.TerrainAltitude(latitude + epsilon, longitude, true) - center),
                Math.Abs(MainBody.TerrainAltitude(latitude - epsilon, longitude, true) - center),
                Math.Abs(MainBody.TerrainAltitude(latitude, longitude + epsilon, true) - center),
                Math.Abs(MainBody.TerrainAltitude(latitude, longitude - epsilon, true) - center)
            }.Max();
            if (MainBody.ocean && center <= 0) return new LandingSiteAssessment(false, slope, roughness, "target is below sea level on an ocean body.");
            if (slope > 15.0) return new LandingSiteAssessment(false, slope, roughness, "local slope exceeds 15 degrees.");
            if (roughness > 15.0) return new LandingSiteAssessment(false, slope, roughness, "local terrain varies by more than 15 m across the footprint sample.");
            return new LandingSiteAssessment(true, slope, roughness, "local terrain accepted: slope " + slope.ToString("F1") + " deg, roughness " + roughness.ToString("F1") + " m.");
        }

        private void SetActiveTarget(double latitude, double longitude, bool traceEvent)
        {
            _activeTargetLatitude = latitude;
            _activeTargetLongitude = longitude;
            _hasActiveTarget = true;
            if (MainBody != null && VesselState != null)
            {
                _targetReferenceUT = VesselState.Time;
                _targetReferencePosition = MainBody.GetWorldSurfacePosition(latitude, longitude, 0) - MainBody.position;
                _hasTargetReference = true;
            }
            if (traceEvent) _pendingTargetEvent = _visualRebaseDone ? "visual_rebase_or_divert" : "target_initialized";
        }

        private void DrawV2MapMarkers()
        {
            if (!ControllerActive || !MapView.MapIsEnabled || MainBody == null || !_hasActiveTarget)
                return;

            // V2 markers are independent of the legacy target-controller marker:
            // red remains the V2 active requested site, blue is only the fresh
            // estimated touchdown, and the original user target is subdued after
            // the one-time local rebase.
            GLUtils.DrawGroundMarker(MainBody, V2ActiveTargetLatitude, V2ActiveTargetLongitude, Color.red, true, 0, MainBody.Radius / 12);
            if (_visualRebaseDone)
                GLUtils.DrawGroundMarker(MainBody, V2OriginalTargetLatitude, V2OriginalTargetLongitude,
                    new Color(0.55f, 0.55f, 0.55f, 0.75f), true, 0, MainBody.Radius / 18);
            LandingGuidanceV2TargetState targetState = TargetState;
            if (!double.IsNaN(targetState.PredictedLatitude) && !double.IsNaN(targetState.PredictedLongitude))
                GLUtils.DrawGroundMarker(MainBody, targetState.PredictedLatitude, targetState.PredictedLongitude,
                    Color.blue, true, 0, MainBody.Radius / 14);
        }

        private sealed class LandingSiteAssessment
        {
            public readonly bool Accepted; public readonly double Slope; public readonly double Roughness; public readonly string Detail;
            public LandingSiteAssessment(bool accepted, double slope, double roughness, string detail)
            { Accepted = accepted; Slope = slope; Roughness = roughness; Detail = detail; }
        }

        private void ReleaseV2Control(bool stopWarp = true)
        {
            if (stopWarp) Core.Warp.MinimumWarp(true);
            Core.Thrust.ThrustOff();
            RestoreV2FineThrustLimits();
            Core.Thrust.Users.Remove(this);
            Core.Attitude.Users.Remove(this);
            Core.Hoverslam.Users.Remove(this);
            Core.GetComputerModule<MechJebModuleLandingPredictions>()?.Users.Remove(this);
            ClearAtmosphericCandidateResult();
            _atmosphericFreshValidationRequested = false;
            _atmosphericPhaseManager = null;
            _v2PoweredStagingGate.Reset();
            _finiteBurnTracking = false;
        }
        private double RemainingFiniteBurnDeltaV => _finiteBurnProgress == null
            ? 0
            : _finiteBurnProgress.RemainingDeltaV;

        private void UpdateFiniteBurnProgress()
        {
            if (!_finiteBurnTracking || _finiteBurnProgress == null || VesselState == null) return;
            // CurrentThrustAcceleration is the measured forward engine
            // acceleration from the completed physics frame. DeltaT is that
            // frame's physical duration, so their product is delivered delta-v.
            _finiteBurnProgress.Integrate(VesselState.DeltaT, VesselState.CurrentThrustAcceleration);
        }

        private void FinishFiniteBurn()
        {
            // TickController integrates exactly once per physics frame before
            // phase dispatch. Do not integrate again here: double-counting the
            // completion frame would corrupt the trace and burn authority.
            _finiteBurnTracking = false;
            Core.Thrust.ThrustOff();
            RestoreV2FineThrustLimits();
        }

        private void CommandFiniteBurnThrottle(double remainingDeltaV)
        {
            Core.Thrust.ThrustForDv(remainingDeltaV, 0.5);
            AirlessFineThrustCommand command = AirlessFineThrustControl.Calculate(remainingDeltaV,
                Core.Thrust.TargetThrottle, VesselState.MinThrustAcceleration, VesselState.MaxThrustAcceleration,
                availableMainThrottle: Core.Thrust.ThrottleLimit);
            Core.Thrust.TargetThrottle = (float)Math.Min(command.RequestedThrottle, Core.Thrust.ThrottleLimit);
            ApplyV2FineThrustLimit(command);
        }

        private void ApplyV2FineThrustLimit(AirlessFineThrustCommand command)
        {
            if (!command.UseEngineThrustLimiter)
            {
                RestoreV2FineThrustLimits();
                return;
            }

            int limitedEngineCount = 0;
            foreach (ModuleEngines engine in Vessel.parts.SelectMany(part => part.Modules.OfType<ModuleEngines>()))
            {
                if (!engine.isOperational || !engine.EngineIgnited) continue;
                if (!_thrustLimitsBeforeV2FineControl.ContainsKey(engine))
                    _thrustLimitsBeforeV2FineControl.Add(engine, engine.thrustPercentage);
                float originalLimit = _thrustLimitsBeforeV2FineControl[engine];
                engine.thrustPercentage = Math.Min(originalLimit,
                    originalLimit * (float)command.RelativeEngineThrustLimit);
                limitedEngineCount++;
            }

            _v2FineEngineCount = limitedEngineCount;
            _v2FineEngineRelativeLimit = limitedEngineCount > 0 ? command.RelativeEngineThrustLimit : 1;
            _v2FineEngineExpectedAcceleration = limitedEngineCount > 0 ? command.ExpectedAcceleration : double.NaN;
        }

        private void RestoreV2FineThrustLimits()
        {
            foreach (KeyValuePair<ModuleEngines, float> entry in _thrustLimitsBeforeV2FineControl)
            {
                if (entry.Key != null) entry.Key.thrustPercentage = entry.Value;
            }
            _thrustLimitsBeforeV2FineControl.Clear();
            _v2FineEngineCount = 0;
            _v2FineEngineRelativeLimit = 1;
            _v2FineEngineExpectedAcceleration = double.NaN;
        }

        private double ThrustVectorAlignmentError(Vector3d commandedVector)
        {
            if (commandedVector.sqrMagnitude < 1e-12 || VesselState == null || VesselState.ThrustForward.sqrMagnitude < 1e-12)
                return double.PositiveInfinity;
            return Vector3d.Angle(VesselState.ThrustForward, commandedVector);
        }

        private bool BurnAlignmentReady(Vector3d commandedVector) =>
            AirlessBurnAlignmentGate.IsReady(Core.Attitude?.attitudeError ?? double.PositiveInfinity,
                ThrustVectorAlignmentError(commandedVector), Vessel?.angularVelocity.magnitude ?? double.PositiveInfinity);

        private double BurnAlignmentError(Vector3d commandedVector) => BurnAlignmentReady(commandedVector)
            ? AirlessBurnAlignmentGate.CombinedError(Core.Attitude?.attitudeError ?? double.PositiveInfinity,
                ThrustVectorAlignmentError(commandedVector))
            : double.PositiveInfinity;

        private void BeginFiniteBurn(string name, double plannedDeltaV)
        {
            // A trim may pause to regain attitude. Preserve its measured
            // delivery when it resumes; resetting it would allow the same burn
            // to spend the planned delta-v repeatedly.
            bool resume = _finiteBurnProgress != null && _phaseBurnName == name &&
                Math.Abs(_phaseBurnPlannedDeltaV - plannedDeltaV) < 0.001;
            _phaseBurnName = name;
            _phaseBurnPlannedDeltaV = plannedDeltaV;
            if (!resume)
            {
                RestoreV2FineThrustLimits();
                _finiteBurnProgress = new FiniteBurnProgress(plannedDeltaV);
            }
            _finiteBurnTracking = true;
        }

        private void TransitionTo(V2FlightPhase next, string status)
        {
            _flightPhase = next;
            if (next == V2FlightPhase.Coast) _terminalWarpGate.Reset();
            ControllerStatus = status;
            if (StructuredTraceEnabled && HighLogic.LoadedSceneIsFlight &&
                (Core.Target.PositionTargetExists || ControllerActive && _hasActiveTarget) && Vessel != null)
            {
                // A phase event must be emitted immediately, but a transition
                // must not synchronously rebuild the estimator/planner a second
                // time after its gate has already produced a fresh preflight.
                if (Preflight != null) WriteCorrelatedTrace(Preflight);
                else RefreshPreflight(true);
            }
        }

        private void SetValidatedAirlessPreflight(LandingGuidanceV2Snapshot snapshot, AirlessLandingPlan plan)
        {
            LandingGuidanceV2Estimate estimate = plan.CandidateEstimate;
            LandingGuidanceV2EstimatorValidation validation =
                AirlessImpactEstimator.ValidateDeterminism(snapshot, estimate);
            double brakingLowerBound = estimate != null && estimate.HasImpact ? estimate.ImpactVelocity.magnitude : double.NaN;
            LandingGuidanceV2PreflightAssessment assessment =
                LandingGuidanceV2PreflightEvaluator.Evaluate(snapshot, estimate, brakingLowerBound);
            Preflight = new LandingGuidanceV2Preflight(snapshot, estimate, validation, assessment, plan,
                brakingLowerBound, Preflight?.AtmosphericPlan);
        }

        public void RefreshPreflight(bool forcePlan = false, bool forceRefresh = false)
        {
            ConsumeAtmosphericCandidateResults();
            ConsumeAtmosphericEntryPlanningResults();
            ConsumeAirlessPlanningResults();
            if (!HighLogic.LoadedSceneIsFlight || (!Core.Target.PositionTargetExists && !(_hasActiveTarget && ControllerActive)) ||
                Vessel == null || MainBody == null)
            {
                Preflight = null;
                return;
            }

            // Estimation, deterministic replay, trace serialization, and any
            // scheduled plan search run at a bounded vessel-time cadence.  The
            // phase manager may call this method on every physics tick, but it
            // must never turn that into synchronous predictor/planner work on
            // every frame.  Burn and warp gates pass forcePlan=true and retain
            // their required fresh snapshot/revalidation semantics.
            if (!forcePlan && !forceRefresh && VesselState.Time < _nextRefreshUT)
                return;
            _nextRefreshUT = VesselState.Time + RefreshInterval;

            LandingGuidanceV2Snapshot snapshot = CaptureSnapshot();
            LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(snapshot);
            LandingGuidanceV2EstimatorValidation estimatorValidation =
                AirlessImpactEstimator.ValidateDeterminism(snapshot, estimate);

            // Cancelling the predicted inertial impact velocity is a physical lower bound,
            // not a landing feasibility claim.  A real plan must additionally account for
            // gravity losses, finite-throttle constraints, terrain, trims, and reserve.
            double brakingLowerBound = estimate.HasImpact ? estimate.ImpactVelocity.magnitude : double.NaN;
            LandingGuidanceV2PreflightAssessment assessment =
                LandingGuidanceV2PreflightEvaluator.Evaluate(snapshot, estimate, brakingLowerBound);
            // Snapshot/estimator diagnostics may run at a short cadence. The
            // complete strategic search is intentionally less frequent in the
            // preview, but every start and command gate forces a plan from this
            // exact snapshot before V2 can request thrust or attitude.
            AirlessLandingPlan airlessPlan;
            // Once V2 has accepted a finite airless burn, it owns that plan.
            // Re-searching for a cheaper future burn every 30 vessel seconds
            // while rails warp is active continuously moves the ignition gate
            // and freezes the Unity thread. The committed vector is instead
            // freshly validated at the burn gate. A new search is permitted
            // only before commitment or at an explicit replan phase such as
            // after a separately executed plane-alignment burn.
            bool retainCommittedAirlessPlan = !forcePlan && !MainBody.atmosphere && ControllerActive &&
                _flightPhase != V2FlightPhase.Preflight && _activePlan != null;
            if (!retainCommittedAirlessPlan &&
                (forcePlan || Preflight?.AirlessPlan == null || VesselState.Time >= _nextPlanRefreshUT))
            {
                if (!_airlessPlanningRunning)
                    StartAirlessPlanning(snapshot);
                airlessPlan = _lastCompletedAirlessPlan ?? AirlessLandingPlanner.Planning(snapshot);
                _nextPlanRefreshUT = VesselState.Time + PlanRefreshInterval;
            }
            else
            {
                airlessPlan = retainCommittedAirlessPlan ? _activePlan :
                    (_lastCompletedAirlessPlan ?? Preflight.AirlessPlan);
            }
            ReentrySimulation.Result atmosphericEstimate = _atmosphericCandidateResult;
            AtmosphericLandingPlan atmosphericPlan;
            bool candidateResultIsCurrent = _atmosphericCandidateResult != null && _atmosphericCandidateSnapshot != null &&
                _atmosphericCandidateSnapshot.Body == snapshot.Body &&
                VesselState.Time <= _nextAtmosphericCandidateSimulationUT;
            bool entryPlanIsCurrent = _lastAtmosphericEntryPlan != null && _lastAtmosphericEntryPlanningSnapshot != null &&
                _lastAtmosphericEntryPlanningSnapshot.Body == snapshot.Body &&
                VesselState.Time <= _lastAtmosphericEntryPlanningSnapshot.UT + PlanRefreshInterval;
            bool refreshAtmosphericPlan = forcePlan || Preflight?.AtmosphericPlan == null ||
                VesselState.Time >= _nextPlanRefreshUT || !ReferenceEquals(atmosphericEstimate, _lastAtmosphericPlanPrediction);
            if (refreshAtmosphericPlan)
            {
                if (candidateResultIsCurrent && _atmosphericCandidatePlan != null)
                    atmosphericPlan = AtmosphericLandingPlanner.Plan(_atmosphericCandidateSnapshot, atmosphericEstimate,
                        _atmosphericCandidatePlan.StrategicEntryDeltaV, _atmosphericCandidatePlan.StrategicEntryBurnUT,
                        _atmosphericCandidatePlan.EntryUT, _atmosphericCandidatePlan.EntryTargetError);
                else if (entryPlanIsCurrent)
                    atmosphericPlan = _lastAtmosphericEntryPlan;
                else
                    atmosphericPlan = new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.WaitingForEstimate,
                        double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                        "V2 is searching a bounded atmospheric entry candidate on a snapshot worker.");
                _lastAtmosphericPlanPrediction = atmosphericEstimate;
                _nextPlanRefreshUT = VesselState.Time + PlanRefreshInterval;
            }
            else
            {
                atmosphericPlan = Preflight.AtmosphericPlan;
            }
            Preflight = new LandingGuidanceV2Preflight(snapshot, estimate, estimatorValidation, assessment, airlessPlan,
                brakingLowerBound, atmosphericPlan);
            if (snapshot.Body.atmosphere && atmosphericPlan.State == AtmosphericLandingPlanState.WaitingForEstimate &&
                atmosphericPlan.StrategicEntryBurnUT > 0 && !_atmosphericCandidateSimulationRunning &&
                (forcePlan || _atmosphericCandidatePlan == null || VesselState.Time >= _nextAtmosphericCandidateSimulationUT))
                StartAtmosphericCandidateSimulation(_lastAtmosphericEntryPlanningSnapshot ?? snapshot, atmosphericPlan);
            else if (snapshot.Body.atmosphere && !entryPlanIsCurrent && !_atmosphericEntryPlanningRunning)
                StartAtmosphericEntryPlanning(snapshot);
            WriteCorrelatedTrace(Preflight);
        }

        // The entry epoch/vector search is bounded, but it still performs
        // enough conic work that it must not run from Unity's flight update.
        // It consumes only the immutable snapshot and returns a candidate
        // which will be simulated and revalidated on the normal V2 gates.
        private void StartAtmosphericEntryPlanning(LandingGuidanceV2Snapshot snapshot)
        {
            _atmosphericEntryPlanningRunning = true;
            long generation = _atmosphericCandidateGeneration;
            ThreadPool.QueueUserWorkItem(RunAtmosphericEntryPlanning,
                new AtmosphericEntryPlanningJob(snapshot, generation));
        }

        private void RunAtmosphericEntryPlanning(object value)
        {
            var job = (AtmosphericEntryPlanningJob)value;
            Stopwatch timer = Stopwatch.StartNew();
            AtmosphericLandingPlan plan;
            try { plan = AtmosphericLandingPlanner.Plan(job.Snapshot, null); }
            catch (Exception ex)
            {
                plan = new AtmosphericLandingPlan(job.Snapshot.Version, AtmosphericLandingPlanState.Rejected,
                    double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                    "V2 atmospheric entry planner worker failed: " + ex.GetType().Name);
            }
            timer.Stop();
            lock (_readyAtmosphericEntryPlanningResults)
                _readyAtmosphericEntryPlanningResults.Enqueue(new AtmosphericEntryPlanningResult(job.Snapshot, plan,
                    job.Generation, timer.Elapsed.TotalMilliseconds));
        }

        private void ConsumeAtmosphericEntryPlanningResults()
        {
            lock (_readyAtmosphericEntryPlanningResults)
            {
                while (_readyAtmosphericEntryPlanningResults.Count > 0)
                {
                    var completed = (AtmosphericEntryPlanningResult)_readyAtmosphericEntryPlanningResults.Dequeue();
                    if (completed.Generation != _atmosphericCandidateGeneration) continue;
                    _atmosphericEntryPlanningRunning = false;
                    _lastAtmosphericEntryPlanningSnapshot = completed.Snapshot;
                    _lastAtmosphericEntryPlan = completed.Plan;
                    _lastAtmosphericEntryPlanningMilliseconds = completed.DurationMilliseconds;
                }
            }
        }

        private sealed class AtmosphericEntryPlanningJob
        {
            public readonly LandingGuidanceV2Snapshot Snapshot;
            public readonly long Generation;
            public AtmosphericEntryPlanningJob(LandingGuidanceV2Snapshot snapshot, long generation)
            { Snapshot = snapshot; Generation = generation; }
        }

        private sealed class AtmosphericEntryPlanningResult
        {
            public readonly LandingGuidanceV2Snapshot Snapshot;
            public readonly AtmosphericLandingPlan Plan;
            public readonly long Generation;
            public readonly double DurationMilliseconds;
            public AtmosphericEntryPlanningResult(LandingGuidanceV2Snapshot snapshot, AtmosphericLandingPlan plan,
                long generation, double durationMilliseconds)
            {
                Snapshot = snapshot;
                Plan = plan;
                Generation = generation;
                DurationMilliseconds = durationMilliseconds;
            }
        }

        private void StartAirlessPlanning(LandingGuidanceV2Snapshot snapshot)
        {
            _airlessPlanningRunning = true;
            long generation = _airlessPlanningGeneration;
            ThreadPool.QueueUserWorkItem(RunAirlessPlanning,
                new AirlessPlanningJob(snapshot, generation));
        }

        private void RunAirlessPlanning(object value)
        {
            var job = (AirlessPlanningJob)value;
            Stopwatch timer = Stopwatch.StartNew();
            AirlessLandingPlan plan;
            try { plan = AirlessLandingPlanner.Plan(job.Snapshot); }
            catch (Exception ex)
            {
                plan = new AirlessLandingPlan(job.Snapshot.Version, AirlessLandingPlanState.Rejected,
                    Vector3d.zero, double.NaN, double.NaN, double.NaN, double.NaN, null,
                    job.Snapshot.AvailableDeltaV, "V2 airless planner worker failed: " + ex.GetType().Name);
            }
            timer.Stop();
            lock (_readyAirlessPlanningResults)
                _readyAirlessPlanningResults.Enqueue(new AirlessPlanningResult(job.Snapshot, plan, job.Generation,
                    timer.Elapsed.TotalMilliseconds));
        }

        private void ConsumeAirlessPlanningResults()
        {
            lock (_readyAirlessPlanningResults)
            {
                while (_readyAirlessPlanningResults.Count > 0)
                {
                    var completed = (AirlessPlanningResult)_readyAirlessPlanningResults.Dequeue();
                    if (completed.Generation != _airlessPlanningGeneration) continue;
                    _airlessPlanningRunning = false;
                    _lastCompletedAirlessPlan = completed.Plan;
                    _lastAirlessPlanningMilliseconds = completed.DurationMilliseconds;
                }
            }
        }

        private sealed class AirlessPlanningJob
        {
            public readonly LandingGuidanceV2Snapshot Snapshot;
            public readonly long Generation;
            public AirlessPlanningJob(LandingGuidanceV2Snapshot snapshot, long generation)
            { Snapshot = snapshot; Generation = generation; }
        }

        private sealed class AirlessPlanningResult
        {
            public readonly LandingGuidanceV2Snapshot Snapshot;
            public readonly AirlessLandingPlan Plan;
            public readonly long Generation;
            public readonly double DurationMilliseconds;
            public AirlessPlanningResult(LandingGuidanceV2Snapshot snapshot, AirlessLandingPlan plan, long generation, double durationMilliseconds)
            { Snapshot = snapshot; Plan = plan; Generation = generation; DurationMilliseconds = durationMilliseconds; }
        }

        // This is deliberately a V2-owned simulation job.  The legacy landing
        // predictor remains an estimator for its own controller; V2 captures a
        // vessel model and conic with the proposed entry burn already applied,
        // then accepts only the result carrying that immutable plan.
        private void StartAtmosphericCandidateSimulation(LandingGuidanceV2Snapshot snapshot, AtmosphericLandingPlan plan)
        {
            try
            {
                var preBurnOrbit = new Orbit();
                preBurnOrbit.UpdateFromStateVectors(snapshot.Position, snapshot.Velocity, snapshot.Body, snapshot.UT);
                Orbit entryOrbit;
                if (plan.StrategicEntryDeltaV.magnitude <= 0.5)
                {
                    entryOrbit = preBurnOrbit;
                }
                else
                {
                    Vector3d position = preBurnOrbit.WorldBCIPositionAtUT(plan.StrategicEntryBurnUT);
                    Vector3d velocity = preBurnOrbit.WorldOrbitalVelocityAtUT(plan.StrategicEntryBurnUT);
                    var burnOrbit = new Orbit();
                    burnOrbit.UpdateFromStateVectors(position, velocity, snapshot.Body, plan.StrategicEntryBurnUT);
                    entryOrbit = burnOrbit.PerturbedOrbit(plan.StrategicEntryBurnUT, plan.StrategicEntryDeltaV);
                }

                MechJebModuleLandingPredictions predictor = Core.GetComputerModule<MechJebModuleLandingPredictions>();
                var curves = ReentrySimulation.SimCurves.Borrow(snapshot.Body);
                var simulatedVessel = SimulatedVessel.Borrow(Vessel, curves, entryOrbit.StartUT, -1);
                var simulation = ReentrySimulation.Borrow(entryOrbit, entryOrbit.StartUT, simulatedVessel, curves,
                    predictor?.descentSpeedPolicy, predictor?.decelEndAltitudeASL ?? 0,
                    VesselState.LimitedMaxThrustAcceleration, predictor?.parachuteSemiDeployMultiplier ?? 3.0,
                    0, false, 0.2, Time.fixedDeltaTime, predictor?.maxOrbits ?? 1.0,
                    predictor?.noSkipToFreefall ?? false);
                _atmosphericCandidateSimulationRunning = true;
                _atmosphericCandidateSnapshot = snapshot;
                _atmosphericCandidatePlan = plan;
                _nextAtmosphericCandidateSimulationUT = VesselState.Time + PlanRefreshInterval;
                ThreadPool.QueueUserWorkItem(RunAtmosphericCandidateSimulation,
                    new AtmosphericCandidateSimulationJob(simulation, snapshot, plan, _atmosphericCandidateGeneration));
            }
            catch (Exception ex)
            {
                _atmosphericCandidateSimulationRunning = false;
                ControllerStatus = "V2 could not start its independent atmospheric candidate simulation: " + ex.Message;
            }
        }

        private void RunAtmosphericCandidateSimulation(object value)
        {
            var job = (AtmosphericCandidateSimulationJob)value;
            try
            {
                ReentrySimulation.Result result = job.Simulation.RunSimulation();
                lock (_readyAtmosphericCandidateResults)
                    _readyAtmosphericCandidateResults.Enqueue(new AtmosphericCandidateSimulationResult(result, job.Snapshot, job.Plan, job.Generation));
            }
            catch (Exception)
            {
                // The worker must not call Unity APIs. Its missing result is
                // treated as an unvalidated candidate on the Unity thread.
            }
            finally
            {
                job.Simulation.Release();
            }
        }

        private void ConsumeAtmosphericCandidateResults()
        {
            lock (_readyAtmosphericCandidateResults)
            {
                while (_readyAtmosphericCandidateResults.Count > 0)
                {
                    var completed = (AtmosphericCandidateSimulationResult)_readyAtmosphericCandidateResults.Dequeue();
                    if (completed.Generation != _atmosphericCandidateGeneration)
                    {
                        completed.Result.Release();
                        continue;
                    }
                    _atmosphericCandidateSimulationRunning = false;
                    if (completed.Result.Outcome == ReentrySimulation.Outcome.ERROR || completed.Result.Body == null)
                    {
                        completed.Result.Release();
                        continue;
                    }
                    completed.Result.EndASL = completed.Result.Body.TerrainAltitude(completed.Result.EndPosition.Latitude,
                        completed.Result.EndPosition.Longitude);
                    if (_atmosphericCandidateResult != null)
                        _atmosphericCandidateResult.Release();
                    _atmosphericCandidateResult = completed.Result;
                    _atmosphericCandidateSnapshot = completed.Snapshot;
                    _atmosphericCandidatePlan = completed.Plan;
                }
            }
        }

        private void ClearAtmosphericCandidateResult()
        {
            _atmosphericCandidateGeneration++;
            if (_atmosphericCandidateResult != null)
            {
                _atmosphericCandidateResult.Release();
                _atmosphericCandidateResult = null;
            }
            _atmosphericCandidateSnapshot = null;
            _atmosphericCandidatePlan = null;
            _atmosphericEntryPlanningRunning = false;
            _lastAtmosphericEntryPlan = null;
            _lastAtmosphericEntryPlanningSnapshot = null;
            _lastAtmosphericEntryPlanningMilliseconds = double.NaN;
            _nextAtmosphericCandidateSimulationUT = 0;
        }

        private sealed class AtmosphericCandidateSimulationJob
        {
            public readonly ReentrySimulation Simulation;
            public readonly LandingGuidanceV2Snapshot Snapshot;
            public readonly AtmosphericLandingPlan Plan;
            public readonly long Generation;
            public AtmosphericCandidateSimulationJob(ReentrySimulation simulation, LandingGuidanceV2Snapshot snapshot, AtmosphericLandingPlan plan, long generation)
            { Simulation = simulation; Snapshot = snapshot; Plan = plan; Generation = generation; }
        }

        private sealed class AtmosphericCandidateSimulationResult
        {
            public readonly ReentrySimulation.Result Result;
            public readonly LandingGuidanceV2Snapshot Snapshot;
            public readonly AtmosphericLandingPlan Plan;
            public readonly long Generation;
            public AtmosphericCandidateSimulationResult(ReentrySimulation.Result result, LandingGuidanceV2Snapshot snapshot, AtmosphericLandingPlan plan, long generation)
            { Result = result; Snapshot = snapshot; Plan = plan; Generation = generation; }
        }

        private LandingGuidanceV2Snapshot CaptureSnapshot()
        {
            Core.StageStats.RequestUpdate();
            double availableDeltaV = Core.StageStats.VacStats.Sum(s => s.DeltaV);
            if (!_hasTargetReference)
            {
                _targetReferenceUT = VesselState.Time;
                _targetReferencePosition = MainBody.GetWorldSurfacePosition(V2ActiveTargetLatitude,
                    V2ActiveTargetLongitude, 0) - MainBody.position;
                _hasTargetReference = true;
            }
            double targetTerrainAltitude;
            try { targetTerrainAltitude = Math.Max(0, MainBody.TerrainAltitude(V2ActiveTargetLatitude, V2ActiveTargetLongitude, true)); }
            catch (Exception) { targetTerrainAltitude = double.NaN; }

            return new LandingGuidanceV2Snapshot(++_snapshotVersion, VesselState.Time, MainBody,
                VesselState.OrbitalPosition, VesselState.OrbitalVelocity, VesselState.Mass, availableDeltaV,
                VesselState.LimitedMaxThrustAcceleration, VesselState.MinThrustAcceleration,
                V2ActiveTargetLatitude, V2ActiveTargetLongitude,
                Vessel.LandedOrSplashed, _targetReferenceUT, _targetReferencePosition, _hasTargetReference, targetTerrainAltitude);
        }

        private void WriteCorrelatedTrace(LandingGuidanceV2Preflight preflight)
        {
            if (!StructuredTraceEnabled || preflight?.Snapshot == null || preflight.Estimate == null)
                return;

            try
            {
                string path = MuUtils.GetCfgPath("LandingGuidanceV2.trace.jsonl");
                MuUtils.FileExistsCreateDirectory(path);
                LandingGuidanceV2Snapshot snapshot = preflight.Snapshot;
                LandingGuidanceV2Estimate estimate = preflight.Estimate;
                LandingGuidanceV2EstimatorValidation estimatorValidation = preflight.EstimatorValidation;
                LandingGuidanceV2PreflightAssessment assessment = preflight.Assessment;
                AirlessLandingPlan airlessPlan = preflight.AirlessPlan;
                AtmosphericLandingPlan atmosphericPlan = preflight.AtmosphericPlan;
                LandingGuidanceV2TargetState targetState = TargetState;
                MechJebModuleLandingAutopilot landing = Core.Landing;
                AutopilotStep step = landing?.CurrentStep;
                string phase = step?.GetType().Name;
                string status = landing?.Status;
                double warpRate = TimeWarp.CurrentRate;

                ReentrySimulation.Result prediction = Core.GetComputerModule<MechJebModuleLandingPredictions>()?.Result;
                if (!ReferenceEquals(prediction, _lastV1Prediction))
                {
                    _lastV1Prediction = prediction;
                    _v1PredictionVersion++;
                }

                string predictionOutcome = prediction?.Outcome.ToString();
                double predictionLat = prediction == null ? double.NaN : prediction.EndPosition.Latitude;
                double predictionLon = prediction == null ? double.NaN : prediction.EndPosition.Longitude;
                double predictionError = prediction == null || prediction.Body == null || !Core.Target.PositionTargetExists || Core.Target.targetBody != prediction.Body
                    ? double.NaN
                    : Vector3d.Distance(
                        prediction.Body.GetWorldSurfacePosition(predictionLat, predictionLon, 0),
                        prediction.Body.GetWorldSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, 0));

                bool landed = Vessel != null && Vessel.LandedOrSplashed;
                bool estimateApplicable = !landed && !snapshot.Body.atmosphere;
                string validityReason = landed ? "landed_or_splashed" : snapshot.Body.atmosphere ? "atmospheric_estimator_not_applicable" : "flight_snapshot";
                Vector3d rcsCommand = Vessel?.ctrlState == null ? Vector3d.zero : new Vector3d(Vessel.ctrlState.X, Vessel.ctrlState.Y, Vessel.ctrlState.Z);
                bool mechjebRcsEnabled = Core.RCS != null && Core.RCS.Enabled;
                bool rcsActionGroupEnabled = Vessel != null && Vessel.ActionGroups[KSPActionGroup.RCS];
                double commandedThrottle = Core.Thrust?.LastThrottle ?? 0;
                double actualThrustAcceleration = VesselState.Mass > 0
                    ? VesselState.ThrustVectorLastFrame.magnitude / VesselState.Mass
                    : double.NaN;
                bool burning = commandedThrottle > 0.001 || actualThrustAcceleration > 0.001;
                double terminalIgnitionUT = Core.Hoverslam?.IgnitionUT ?? double.NaN;
                double terminalIgnitionCountdown = Core.Hoverslam?.IgnitionCountdown ?? double.NaN;
                bool terminalIgnitionAvailable = !double.IsNaN(terminalIgnitionUT) &&
                    !double.IsInfinity(terminalIgnitionUT) && !double.IsNaN(terminalIgnitionCountdown) &&
                    !double.IsInfinity(terminalIgnitionCountdown);
                string v1Phase = JsonString(phase);
                string v1Status = JsonString(status);
                string predOutcome = JsonString(predictionOutcome);
                string baseFields = string.Format(CultureInfo.InvariantCulture,
                    "\"snapshotVersion\":{0},\"ut\":{1},\"body\":\"{2}\",\"targetLat\":{3},\"targetLon\":{4}," +
                    "\"position\":[{5},{6},{7}],\"velocity\":[{8},{9},{10}],\"mass\":{11}," +
                    "\"availableDeltaV\":{12},\"maxAcceleration\":{13},\"minAcceleration\":{14},\"outcome\":\"{15}\",\"impactUT\":{16}," +
                    "\"targetError\":{17},\"brakingDeltaVLowerBound\":{18},\"deltaVAboveLowerBound\":{19}," +
                    "\"v1Phase\":{20},\"v1Status\":{21},\"v1PredictionVersion\":{22},\"v1PredictionOutcome\":{23}," +
                    "\"v1PredictionEndLat\":{24},\"v1PredictionEndLon\":{25},\"v1PredictionEndUT\":{26},\"v1TargetError\":{27}," +
                    "\"warpRate\":{28},\"attitudeErrorDegrees\":{29},\"commandedThrottle\":{30},\"flightControlThrottle\":{31}," +
                    "\"actualThrustAcceleration\":{32},\"forwardThrustAcceleration\":{33},\"mechjebRcsEnabled\":{34}," +
                    "\"rcsActionGroupEnabled\":{35},\"rcsCommand\":[{36},{37},{38}],\"burnActive\":{39}," +
                    "\"isLandedOrSplashed\":{40},\"estimatorApplicable\":{41},\"flightDataValid\":{42},\"validityReason\":{43}," +
                    "\"estimatorRepeatOutcome\":{44},\"estimatorRepeatImpactUT\":{45},\"estimatorDeterministic\":{46},\"estimatorValidationDetail\":{47}," +
                    "\"preflightState\":{48},\"preflightLocalGravity\":{49},\"preflightReason\":{50}," +
                    "\"airlessPlanState\":{51},\"strategicDeorbitDeltaV\":{52},\"planTerminalLowerBound\":{53},\"planLowerBoundMargin\":{54}," +
                    "\"planDownrange\":{55},\"planCrossRange\":{56},\"planCorridorLimit\":{57},\"planReason\":{58},\"v2CommandAuthorized\":{59},\"v2Phase\":{60},\"v2Status\":{61},\"v2AutoWarp\":{62},\"planPlaneBurnUT\":{63},\"planStrategicBurnUT\":{64},\"planBrakingEntryUT\":{65}",
                    snapshot.Version, JsonNumber(snapshot.UT), EscapeJson(snapshot.Body.bodyName), JsonNumber(snapshot.TargetLatitude),
                    JsonNumber(snapshot.TargetLongitude), JsonNumber(snapshot.Position.x), JsonNumber(snapshot.Position.y),
                    JsonNumber(snapshot.Position.z), JsonNumber(snapshot.Velocity.x), JsonNumber(snapshot.Velocity.y),
                    JsonNumber(snapshot.Velocity.z), JsonNumber(snapshot.Mass), JsonNumber(snapshot.AvailableDeltaV),
                    JsonNumber(snapshot.MaximumAcceleration), JsonNumber(snapshot.MinimumAcceleration), estimate.Outcome, JsonNumber(estimate.ImpactUT), JsonNumber(estimate.TargetError),
                    JsonNumber(preflight.BrakingDeltaVLowerBound), JsonNumber(preflight.DeltaVAboveLowerBound), v1Phase, v1Status,
                    _v1PredictionVersion, predOutcome, JsonNumber(predictionLat), JsonNumber(predictionLon),
                    JsonNumber(prediction?.EndUT ?? double.NaN), JsonNumber(predictionError), JsonNumber(warpRate),
                    JsonNumber(Core.Attitude?.attitudeError ?? double.NaN), JsonNumber(commandedThrottle),
                    JsonNumber(Vessel?.ctrlState?.mainThrottle ?? float.NaN), JsonNumber(actualThrustAcceleration),
                    JsonNumber(VesselState.CurrentThrustAcceleration), mechjebRcsEnabled ? "true" : "false",
                    rcsActionGroupEnabled ? "true" : "false", JsonNumber(rcsCommand.x), JsonNumber(rcsCommand.y),
                    JsonNumber(rcsCommand.z), burning ? "true" : "false",
                    landed ? "true" : "false", estimateApplicable ? "true" : "false", !landed ? "true" : "false", JsonString(validityReason),
                    JsonString(estimatorValidation?.RepeatedEstimate?.Outcome.ToString()),
                    JsonNumber(estimatorValidation?.RepeatedEstimate?.ImpactUT ?? double.NaN),
                    estimatorValidation != null && estimatorValidation.IsDeterministic ? "true" : "false",
                    JsonString(estimatorValidation?.Detail), JsonString(assessment?.State.ToString()),
                    JsonNumber(assessment?.LocalGravity ?? double.NaN), JsonString(assessment?.Reason),
                    JsonString(airlessPlan?.State.ToString()), JsonNumber(airlessPlan?.StrategicDeorbitDeltaVMagnitude ?? double.NaN),
                    JsonNumber(airlessPlan?.TerminalBrakingLowerBound ?? double.NaN), JsonNumber(airlessPlan?.LowerBoundMargin ?? double.NaN),
                    JsonNumber(airlessPlan?.SignedDownrange ?? double.NaN), JsonNumber(airlessPlan?.CrossRange ?? double.NaN),
                    JsonNumber(airlessPlan?.CorridorLimit ?? double.NaN), JsonString(airlessPlan?.Reason),
                    (airlessPlan != null && airlessPlan.CommandAuthorized || atmosphericPlan?.State == AtmosphericLandingPlanState.Candidate) ? "true" : "false",
                    JsonString(_flightPhase.ToString()), JsonString(ControllerStatus), V2AutoWarp ? "true" : "false",
                    JsonNumber(airlessPlan?.PlaneAlignmentBurnUT ?? double.NaN), JsonNumber(airlessPlan?.StrategicBurnUT ?? double.NaN),
                    JsonNumber(airlessPlan?.BrakingEntryUT ?? double.NaN));
                baseFields += string.Format(CultureInfo.InvariantCulture,
                    ",\"targetReferenceUT\":{0},\"targetReferencePosition\":[{1},{2},{3}],\"targetTerrainAltitude\":{4},\"hasTargetReferencePosition\":{5}",
                    JsonNumber(snapshot.TargetReferenceUT), JsonNumber(snapshot.TargetReferencePosition.x),
                    JsonNumber(snapshot.TargetReferencePosition.y), JsonNumber(snapshot.TargetReferencePosition.z),
                    JsonNumber(snapshot.TargetTerrainAltitude), snapshot.HasTargetReferencePosition ? "true" : "false");
                baseFields += string.Format(CultureInfo.InvariantCulture,
                    ",\"atmosphericPlanState\":{0},\"atmosphericTargetError\":{1},\"atmosphericEntryCorridor\":{2},\"atmosphericEndpointUncertainty\":{3},\"atmosphericTerminalReserve\":{4},\"atmosphericLandingMargin\":{5},\"atmosphericPlanReason\":{6},\"atmosphericStrategicEntryDeltaV\":{7},\"atmosphericStrategicEntryBurnUT\":{8},\"atmosphericEntryUT\":{9},\"atmosphericEntryTargetError\":{10}",
                    JsonString(atmosphericPlan?.State.ToString()), JsonNumber(atmosphericPlan?.PredictedTargetError ?? double.NaN),
                    JsonNumber(atmosphericPlan?.EntryCorridorRadius ?? double.NaN), JsonNumber(atmosphericPlan?.EndpointUncertainty ?? double.NaN),
                    JsonNumber(atmosphericPlan?.TerminalReserve ?? double.NaN), JsonNumber(atmosphericPlan?.LandingMargin ?? double.NaN),
                    JsonString(atmosphericPlan?.Reason), JsonNumber(atmosphericPlan?.StrategicEntryDeltaV.magnitude ?? double.NaN),
                    JsonNumber(atmosphericPlan?.StrategicEntryBurnUT ?? double.NaN), JsonNumber(atmosphericPlan?.EntryUT ?? double.NaN),
                    JsonNumber(atmosphericPlan?.EntryTargetError ?? double.NaN));
                baseFields += ",\"terrainAltitude\":" + JsonNumber(estimate.TerrainAltitude);
                baseFields += string.Format(CultureInfo.InvariantCulture,
                    ",\"atmosphericCandidateSimulationRunning\":{0},\"atmosphericCandidateOutcome\":{1},\"atmosphericCandidateSnapshotVersion\":{2},\"atmosphericCandidatePlanSnapshotVersion\":{3}",
                    _atmosphericCandidateSimulationRunning ? "true" : "false",
                    JsonString(_atmosphericCandidateResult?.Outcome.ToString()), _atmosphericCandidateSnapshot?.Version ?? -1,
                    _atmosphericCandidatePlan?.SnapshotVersion ?? -1);
                baseFields += string.Format(CultureInfo.InvariantCulture,
                    ",\"v2OriginalTargetLat\":{0},\"v2OriginalTargetLon\":{1},\"v2ActiveTargetLat\":{2},\"v2ActiveTargetLon\":{3},\"v2PredictedTargetLat\":{4},\"v2PredictedTargetLon\":{5},\"v2PredictionSnapshotVersion\":{6},\"v2VisualRebaseDone\":{7},\"v2SiteAccepted\":{8},\"v2SiteSlopeDegrees\":{9},\"v2SiteRoughness\":{10},\"v2SiteDetail\":{11},\"v2FiniteBurn\":{12},\"v2PlannedBurnDeltaV\":{13},\"v2DeliveredBurnDeltaV\":{14}",
                    JsonNumber(targetState.OriginalLatitude), JsonNumber(targetState.OriginalLongitude),
                    JsonNumber(targetState.ActiveLatitude), JsonNumber(targetState.ActiveLongitude),
                    JsonNumber(targetState.PredictedLatitude), JsonNumber(targetState.PredictedLongitude), targetState.PredictionSnapshotVersion,
                    targetState.VisualRebaseDone ? "true" : "false",
                    _siteAssessment != null && _siteAssessment.Accepted ? "true" : "false", JsonNumber(_siteAssessment?.Slope ?? double.NaN),
                    JsonNumber(_siteAssessment?.Roughness ?? double.NaN), JsonString(_siteAssessment?.Detail), JsonString(_phaseBurnName),
                    JsonNumber(_phaseBurnPlannedDeltaV), JsonNumber(_finiteBurnProgress?.DeliveredDeltaV ?? double.NaN));
                double thrustVectorError = ThrustVectorAlignmentError(_commandedV2AttitudeVector);
                baseFields += string.Format(CultureInfo.InvariantCulture,
                    ",\"v2CommandedAttitudeVector\":[{0},{1},{2}],\"v2MeasuredThrustForward\":[{3},{4},{5}],\"v2ThrustVectorAlignmentErrorDegrees\":{6},\"v2AngularVelocityRadiansPerSecond\":{7},\"v2BurnAlignmentReady\":{8}",
                    JsonNumber(_commandedV2AttitudeVector.x), JsonNumber(_commandedV2AttitudeVector.y), JsonNumber(_commandedV2AttitudeVector.z),
                    JsonNumber(VesselState.ThrustForward.x), JsonNumber(VesselState.ThrustForward.y), JsonNumber(VesselState.ThrustForward.z),
                    JsonNumber(thrustVectorError), JsonNumber(Vessel?.angularVelocity.magnitude ?? double.NaN),
                    BurnAlignmentReady(_commandedV2AttitudeVector) ? "true" : "false");
                baseFields += string.Format(CultureInfo.InvariantCulture,
                    ",\"v2EngineThrustLimiterActive\":{0},\"v2EngineThrustLimiterRelative\":{1},\"v2EngineThrustLimiterExpectedAcceleration\":{2},\"v2EngineThrustLimiterEngineCount\":{3}",
                    _v2FineEngineCount > 0 ? "true" : "false", JsonNumber(_v2FineEngineRelativeLimit),
                    JsonNumber(_v2FineEngineExpectedAcceleration), _v2FineEngineCount);
                baseFields += string.Format(CultureInfo.InvariantCulture,
                    ",\"v2TerminalIgnitionAvailable\":{0},\"v2TerminalIgnitionUT\":{1},\"v2TerminalIgnitionCountdown\":{2},\"v2TerminalWarpAttitudeReady\":{3}",
                    terminalIgnitionAvailable ? "true" : "false", JsonNumber(terminalIgnitionUT),
                    JsonNumber(terminalIgnitionCountdown),
                    BurnAlignmentReady(Core.Hoverslam?.IgnitionAttitude ?? Vector3d.zero) ? "true" : "false");
                baseFields += string.Format(CultureInfo.InvariantCulture,
                    ",\"v2CurrentStage\":{0},\"v2LastStageUT\":{1}",
                    Vessel?.currentStage ?? -1, JsonNumber(_lastV2StageUT));
                baseFields += string.Format(CultureInfo.InvariantCulture,
                    ",\"airlessPlanSnapshotVersion\":{0},\"activeAirlessPlanSnapshotVersion\":{1},\"atmosphericPlanSnapshotVersion\":{2},\"airlessPlanningDurationMilliseconds\":{3},\"atmosphericEntryPlanningDurationMilliseconds\":{4},\"atmosphericEntryPlanningRunning\":{5},\"phaseManagerWorkUnits\":{6}",
                    airlessPlan?.SnapshotVersion ?? -1, _activePlan?.SnapshotVersion ?? -1,
                    atmosphericPlan?.SnapshotVersion ?? -1, JsonNumber(_lastAirlessPlanningMilliseconds),
                    JsonNumber(_lastAtmosphericEntryPlanningMilliseconds), _atmosphericEntryPlanningRunning ? "true" : "false",
                    _airlessPhaseManager.LastWorkUnits);

                var lines = new System.Collections.Generic.List<string>();
                if (_lastV1Phase != phase)
                    lines.Add(TraceRecord(baseFields, "phase_transition", _lastV1Phase, phase));
                string v2Phase = _flightPhase.ToString();
                if (_lastV2Phase != v2Phase)
                    lines.Add(TraceRecord(baseFields, "v2_phase_transition", _lastV2Phase, v2Phase));
                if (_lastV1Burning != burning && (_lastV1Burning.HasValue || burning))
                    lines.Add(TraceRecord(baseFields, burning ? "burn_start" : "burn_end", _lastV1Burning.HasValue && _lastV1Burning.Value ? "burning" : "not_burning", burning ? "burning" : "not_burning"));
                bool warped = warpRate > 1.0;
                if (_lastWarped != warped && (_lastWarped.HasValue || warped))
                    lines.Add(TraceRecord(baseFields, warped ? "warp_enter" : "warp_exit", _lastWarped.HasValue && _lastWarped.Value ? "warped" : "1x", warped ? "warped" : "1x"));
                if (_pendingTargetEvent != null)
                {
                    lines.Add(TraceRecord(baseFields, _pendingTargetEvent, null, _flightPhase.ToString()));
                    _pendingTargetEvent = null;
                }
                _lastV1Phase = phase;
                _lastV2Phase = v2Phase;
                _lastV1Burning = burning;
                _lastWarped = warped;
                lines.Add("{\"recordType\":\"sample\"," + baseFields + "}");
                File.AppendAllText(path, string.Join(Environment.NewLine, lines) + Environment.NewLine);
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[MechJebLandingGuidanceV2] Could not write structured trace: " + ex.GetType().Name);
            }
        }

        private static string TraceRecord(string fields, string eventName, string from, string to) =>
            "{\"recordType\":\"event\",\"event\":\"" + EscapeJson(eventName) + "\",\"eventFrom\":" + JsonString(from) +
            ",\"eventTo\":" + JsonString(to) + "," + fields + "}";

        private static string JsonString(string value) => value == null ? "null" : "\"" + EscapeJson(value) + "\"";

        private static string EscapeJson(string value) => (value ?? string.Empty)
            .Replace("\\", "\\\\")
            .Replace("\"", "\\\"")
            .Replace("\r", "\\r")
            .Replace("\n", "\\n")
            .Replace("\t", "\\t");

        private static string JsonNumber(double value) => double.IsNaN(value) || double.IsInfinity(value)
            ? "null"
            : value.ToString("F6", CultureInfo.InvariantCulture);
    }
}
