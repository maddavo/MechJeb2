extern alias JetBrainsAnnotations;
using System;
using System.Globalization;
using System.IO;
using System.Linq;
using JetBrainsAnnotations::JetBrains.Annotations;
using MechJebLib.Control;
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
        private long _snapshotVersion;
        private long _v1PredictionVersion;
        private ReentrySimulation.Result _lastV1Prediction;
        private string _lastV1Phase;
        private bool? _lastV1Burning;
        private bool? _lastWarped;
        private V2FlightPhase _flightPhase;
        private AirlessLandingPlan _activePlan;
        private Vector3d _burnTargetVelocity;
        private Vector3d _trimDeltaV;
        private Vector3d _lastAdjustedVelocity;
        private Vector3d _phaseBurnStartVelocity;
        private double _phaseBurnPlannedDeltaV;
        private string _phaseBurnName;
        private string _lastV2Phase;
        private double _activeTargetLatitude;
        private double _activeTargetLongitude;
        private double _selectedTargetLatitude;
        private double _selectedTargetLongitude;
        private double _originalTargetLatitude;
        private double _originalTargetLongitude;
        private bool _hasActiveTarget;
        private bool _visualRebaseDone;
        private LandingSiteAssessment _siteAssessment;
        private string _pendingTargetEvent;
        private const double VisualAssessmentAltitude = 750.0;
        private const double VisualRebaseAccuracyLimit = 500.0;
        private readonly DeltaSigmaThrottleModulator _terminalPwm = new DeltaSigmaThrottleModulator(0.02, 0.50);

        public enum V2FlightPhase { Idle, Preflight, WarpToStrategic, AlignPlane, PlaneAlignment, AlignStrategicBurn, StrategicBurn, AlignTrim, BoundedTrim, Coast, AtmosphericEntry, BrakingApproach, VisualAssessment, TerminalDivert, VelocityNull, Complete, Rejected }

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool PreviewEnabled;

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool StructuredTraceEnabled;

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool V2AutoWarp = true;

        public LandingGuidanceV2Preflight Preflight { get; private set; }

        public bool IsPreviewOnly => _flightPhase == V2FlightPhase.Idle || _flightPhase == V2FlightPhase.Rejected || _flightPhase == V2FlightPhase.Complete;
        public bool ControllerActive => _flightPhase == V2FlightPhase.Preflight || _flightPhase == V2FlightPhase.WarpToStrategic || _flightPhase == V2FlightPhase.AlignPlane || _flightPhase == V2FlightPhase.PlaneAlignment || _flightPhase == V2FlightPhase.AlignStrategicBurn || _flightPhase == V2FlightPhase.StrategicBurn || _flightPhase == V2FlightPhase.AlignTrim || _flightPhase == V2FlightPhase.BoundedTrim || _flightPhase == V2FlightPhase.Coast || _flightPhase == V2FlightPhase.AtmosphericEntry || _flightPhase == V2FlightPhase.BrakingApproach || _flightPhase == V2FlightPhase.VisualAssessment || _flightPhase == V2FlightPhase.TerminalDivert || _flightPhase == V2FlightPhase.VelocityNull;
        public V2FlightPhase FlightPhase => _flightPhase;
        public string ControllerStatus { get; private set; } = "Idle";
        public bool HasV2ActiveTarget => _hasActiveTarget;
        public double V2ActiveTargetLatitude => _hasActiveTarget ? _activeTargetLatitude : (double)Core.Target.targetLatitude;
        public double V2ActiveTargetLongitude => _hasActiveTarget ? _activeTargetLongitude : (double)Core.Target.targetLongitude;
        public double V2OriginalTargetLatitude => _hasActiveTarget ? _originalTargetLatitude : (double)Core.Target.targetLatitude;
        public double V2OriginalTargetLongitude => _hasActiveTarget ? _originalTargetLongitude : (double)Core.Target.targetLongitude;
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
            if (PreviewEnabled && HighLogic.LoadedSceneIsFlight && Core.Target.PositionTargetExists && VesselState.Time >= _nextRefreshUT)
            {
                _nextRefreshUT = VesselState.Time + RefreshInterval;
                RefreshPreflight();
            }
        }

        public bool StartLanding()
        {
            if (!HighLogic.LoadedSceneIsFlight || !Core.Target.PositionTargetExists)
                return RejectController("Select a landing target before starting V2.");
            SetActiveTarget(Core.Target.targetLatitude, Core.Target.targetLongitude, false);
            _originalTargetLatitude = Core.Target.targetLatitude;
            _originalTargetLongitude = Core.Target.targetLongitude;
            _selectedTargetLatitude = Core.Target.targetLatitude;
            _selectedTargetLongitude = Core.Target.targetLongitude;
            _visualRebaseDone = false;
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
            if (Preflight?.AirlessPlan == null || Preflight.AirlessPlan.State != AirlessLandingPlanState.Candidate) return RejectController("V2 requires a current airless strategic-deorbit candidate.");
            _activePlan = Preflight.AirlessPlan;
            Core.Thrust.Users.Add(this); Core.Attitude.Users.Add(this);
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

        private bool RejectController(string reason) { ReleaseV2Control(); TransitionTo(V2FlightPhase.Rejected, reason); return false; }

        private void TickController()
        {
            if (Vessel == null || Vessel.LandedOrSplashed) { ReleaseV2Control(); TransitionTo(V2FlightPhase.Complete, "V2 landing completed: vessel is landed or splashed."); return; }
            if (Core.Landing != null && Core.Landing.Enabled) { RejectController("V1 Landing Guidance was engaged; V2 relinquished control."); return; }
            SyncSelectedTarget();
            switch (_flightPhase)
            {
                case V2FlightPhase.Preflight:
                    Core.Thrust.ThrustOff();
                    Core.Warp.MinimumWarp(true);
                    RefreshPreflight(true);
                    if (MainBody.atmosphere)
                    {
                        if (Preflight?.AtmosphericPlan?.State == AtmosphericLandingPlanState.Candidate)
                            TransitionTo(V2FlightPhase.AtmosphericEntry,
                                "V2 atmospheric entry corridor is validated; holding the independent entry profile.");
                        else if (Preflight?.AtmosphericPlan?.State == AtmosphericLandingPlanState.Rejected)
                            RejectController(Preflight.AtmosphericPlan.Reason);
                    }
                    break;
                case V2FlightPhase.WarpToStrategic:
                    Core.Thrust.ThrustOff();
                    double nextBurnUT = _activePlan.PlaneAlignmentDeltaVMagnitude > 0.5
                        ? _activePlan.PlaneAlignmentBurnUT
                        : _activePlan.StrategicBurnUT;
                    if (VesselState.Time < nextBurnUT - 20.0 && V2AutoWarp)
                    {
                        Core.Warp.WarpToUT(nextBurnUT - 20.0);
                        break;
                    }
                    Core.Warp.MinimumWarp(true);
                    RefreshPreflight(true);
                    if (Preflight?.AirlessPlan == null || Preflight.AirlessPlan.State != AirlessLandingPlanState.Candidate)
                    {
                        RejectController("V2 plan failed fresh validation at the strategic-burn gate.");
                        break;
                    }
                    _activePlan = Preflight.AirlessPlan;
                    nextBurnUT = _activePlan.PlaneAlignmentDeltaVMagnitude > 0.5
                        ? _activePlan.PlaneAlignmentBurnUT
                        : _activePlan.StrategicBurnUT;
                    if (nextBurnUT > VesselState.Time + 25.0)
                        break;
                    _burnTargetVelocity = VesselState.OrbitalVelocity + _activePlan.PlaneAlignmentDeltaV;
                    TransitionTo(_activePlan.PlaneAlignmentDeltaVMagnitude > 0.5 ? V2FlightPhase.AlignPlane : V2FlightPhase.AlignStrategicBurn,
                        _activePlan.PlaneAlignmentDeltaVMagnitude > 0.5 ? "Aligning for the validated V2 plane-alignment burn." : "Aligning for the validated V2 strategic deorbit burn.");
                    break;
                case V2FlightPhase.AlignPlane:
                    Core.Thrust.ThrustOff();
                    Core.Attitude.attitudeTo(_activePlan.PlaneAlignmentDeltaV, AttitudeReference.INERTIAL_COT, this);
                    if (Core.Attitude.attitudeError < 2.0)
                    {
                        BeginFiniteBurn("plane_alignment", _activePlan.PlaneAlignmentDeltaVMagnitude);
                        TransitionTo(V2FlightPhase.PlaneAlignment, "Executing the finite V2 plane-alignment burn.");
                    }
                    break;
                case V2FlightPhase.PlaneAlignment:
                    Core.Attitude.attitudeTo(_activePlan.PlaneAlignmentDeltaV, AttitudeReference.INERTIAL_COT, this);
                    if (Vector3d.Dot(_burnTargetVelocity - VesselState.OrbitalVelocity, _activePlan.PlaneAlignmentDeltaV.normalized) <= 0.5)
                    {
                        Core.Thrust.ThrustOff();
                        RefreshPreflight(true);
                        if (Preflight?.AirlessPlan == null || Preflight.AirlessPlan.State != AirlessLandingPlanState.Candidate)
                        {
                            RejectController("V2 plan failed fresh validation after plane alignment.");
                            break;
                        }
                        _activePlan = Preflight.AirlessPlan;
                        TransitionTo(V2FlightPhase.WarpToStrategic, "Plane alignment complete; V2 is revalidating the strategic-deorbit gate.");
                    }
                    else Core.Thrust.ThrustForDv(Vector3d.Dot(_burnTargetVelocity - VesselState.OrbitalVelocity, _activePlan.PlaneAlignmentDeltaV.normalized), 0.5);
                    break;
                case V2FlightPhase.AlignStrategicBurn:
                    Core.Thrust.ThrustOff(); Core.Attitude.attitudeTo(_activePlan.StrategicDeorbitDeltaV, AttitudeReference.INERTIAL_COT, this);
                    if (Core.Attitude.attitudeError < 2.0)
                    {
                        BeginFiniteBurn("strategic_deorbit", _activePlan.StrategicDeorbitDeltaVMagnitude);
                        TransitionTo(V2FlightPhase.StrategicBurn, "Executing V2 strategic deorbit burn.");
                    }
                    break;
                case V2FlightPhase.StrategicBurn:
                    Core.Attitude.attitudeTo(_activePlan.StrategicDeorbitDeltaV, AttitudeReference.INERTIAL_COT, this);
                    double remainingDv = Vector3d.Dot(_burnTargetVelocity - VesselState.OrbitalVelocity, _activePlan.StrategicDeorbitDeltaV.normalized);
                    if (remainingDv <= 0.5)
                    {
                        Core.Thrust.ThrustOff();
                        RefreshPreflight(true);
                        if (Preflight?.Estimate == null || !Preflight.Estimate.HasImpact)
                        {
                            RejectController("V2 strategic burn did not produce a fresh valid impact trajectory.");
                            break;
                        }
                        LandingGuidanceV2Snapshot trimSnapshot = CaptureSnapshot();
                        if (Preflight.Estimate.TargetError > _activePlan.CorridorLimit &&
                            AirlessLandingPlanner.TryPlanBoundedTrim(trimSnapshot, _activePlan.TrimBudget, out _trimDeltaV, out LandingGuidanceV2Estimate trimEstimate))
                        {
                            _burnTargetVelocity = VesselState.OrbitalVelocity + _trimDeltaV;
                            TransitionTo(V2FlightPhase.AlignTrim, "A bounded V2 trim improved the fresh target estimate.");
                        }
                        else if (Preflight.Estimate.TargetError <= _activePlan.CorridorLimit)
                            TransitionTo(V2FlightPhase.Coast, "Strategic deorbit complete; coasting to V2 braking approach.");
                        else
                            RejectController("V2 target error is outside the corridor and no bounded trim is feasible.");
                    }
                    else Core.Thrust.ThrustForDv(remainingDv, 0.5);
                    break;
                case V2FlightPhase.AlignTrim:
                    Core.Thrust.ThrustOff();
                    Core.Attitude.attitudeTo(_trimDeltaV, AttitudeReference.INERTIAL_COT, this);
                    if (Core.Attitude.attitudeError < 2.0)
                    {
                        BeginFiniteBurn("bounded_trim", _trimDeltaV.magnitude);
                        TransitionTo(V2FlightPhase.BoundedTrim, "Executing one bounded V2 trim burn.");
                    }
                    break;
                case V2FlightPhase.BoundedTrim:
                    Core.Attitude.attitudeTo(_trimDeltaV, AttitudeReference.INERTIAL_COT, this);
                    if (Vector3d.Dot(_burnTargetVelocity - VesselState.OrbitalVelocity, _trimDeltaV.normalized) <= 0.25)
                    {
                        Core.Thrust.ThrustOff();
                        RefreshPreflight();
                        if (Preflight?.Estimate == null || !Preflight.Estimate.HasImpact)
                        {
                            RejectController("V2 trim did not retain a valid impact trajectory.");
                            break;
                        }
                        if (Preflight.Estimate.TargetError > _activePlan.CorridorLimit)
                            RejectController("V2 trim did not bring the target estimate inside the accepted corridor.");
                        else
                            TransitionTo(V2FlightPhase.Coast, "Bounded V2 trim complete; coasting to braking approach.");
                    }
                    else Core.Thrust.ThrustForDv(Vector3d.Dot(_burnTargetVelocity - VesselState.OrbitalVelocity, _trimDeltaV.normalized), 0.5);
                    break;
                case V2FlightPhase.Coast:
                    Core.Thrust.ThrustOff();
                    if (double.IsNaN(Core.Hoverslam.IgnitionUT) || double.IsInfinity(Core.Hoverslam.IgnitionUT))
                    {
                        RejectController("V2 terminal-braking simulation has no ignition solution.");
                        break;
                    }
                    Core.Attitude.attitudeTo(Core.Hoverslam.IgnitionAttitude, AttitudeReference.INERTIAL_COT, this);
                    if (V2AutoWarp && VesselState.Time < Core.Hoverslam.IgnitionUT - 10.0)
                        Core.Warp.WarpToUT(Core.Hoverslam.IgnitionUT - 10.0);
                    else if (Core.Hoverslam.IgnitionCountdown <= Time.fixedDeltaTime)
                    {
                        Core.Warp.MinimumWarp(true);
                        _lastAdjustedVelocity = new Vector3d(double.NaN, double.NaN, double.NaN);
                        TransitionTo(V2FlightPhase.BrakingApproach, "V2 braking approach has begun.");
                    }
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
                    RefreshPreflight(false);
                    if (Preflight?.AtmosphericPlan?.State == AtmosphericLandingPlanState.Rejected)
                    {
                        RejectController(Preflight.AtmosphericPlan.Reason);
                        break;
                    }
                    if (!double.IsNaN(Core.Hoverslam.IgnitionUT) && !double.IsInfinity(Core.Hoverslam.IgnitionUT) &&
                        Core.Hoverslam.IgnitionCountdown <= 5.0)
                        TransitionTo(V2FlightPhase.BrakingApproach, "V2 atmospheric powered braking has begun.");
                    break;
                case V2FlightPhase.BrakingApproach:
                    if (VesselState.AltitudeBottom <= VisualAssessmentAltitude && !_visualRebaseDone)
                    {
                        Core.Warp.MinimumWarp(true);
                        TransitionTo(V2FlightPhase.VisualAssessment, "V2 entered the local visual-assessment gate.");
                        break;
                    }
                    Vector3d adjustedVelocity = TerminalVelocityError();
                    Core.Attitude.attitudeTo(-adjustedVelocity, AttitudeReference.INERTIAL_COT, this);
                    Core.Thrust.TargetThrottle = 1.0f;
                    if (!double.IsNaN(_lastAdjustedVelocity.x) && Vector3d.Angle(_lastAdjustedVelocity, adjustedVelocity) > 10.0)
                    {
                        _terminalPwm.Reset();
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
        }

        private void TickTerminalDivert()
        {
            Vector3d velocityError = TerminalVelocityError();
            Vector3d target = MainBody.GetWorldSurfacePosition(V2ActiveTargetLatitude, V2ActiveTargetLongitude,
                MainBody.TerrainAltitude(V2ActiveTargetLatitude, V2ActiveTargetLongitude, true));
            double horizontalError = Vector3d.Exclude(VesselState.Up, target - VesselState.CoM).magnitude;
            if (horizontalError < 5.0 && VesselState.AltitudeBottom < 50.0)
            {
                _terminalPwm.Reset();
                TransitionTo(V2FlightPhase.VelocityNull, "V2 velocity-null guidance is protecting touchdown speed and clearance.");
                return;
            }
            if (Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up) >= -1.0)
                Core.Attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, this);
            else
                Core.Attitude.attitudeTo(-velocityError, AttitudeReference.INERTIAL_COT, this);
            double altitude = Math.Max(0.1, VesselState.AltitudeBottom);
            double acceleration = Vessel.graviticAcceleration.magnitude + 0.5 * (VesselState.SurfaceVelocity.sqrMagnitude - 0.25) / altitude;
            _terminalPwm.MinOnTime = 0.50;
            _terminalPwm.MinOffTime = TimeWarp.fixedDeltaTime;
            Core.Thrust.TargetThrottle = _terminalPwm.ThrottleCommand(acceleration, VesselState.MinThrustAcceleration, VesselState.MaxThrustAcceleration, TimeWarp.fixedDeltaTime);
        }

        private void TickVelocityNull()
        {
            Core.Warp.MinimumWarp(true);
            Core.Attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, this);
            double altitude = Math.Max(0.1, VesselState.AltitudeBottom);
            double verticalSpeed = Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up);
            double desiredAcceleration = Vessel.graviticAcceleration.magnitude + Math.Max(0, (verticalSpeed * verticalSpeed - 0.25) / (2.0 * altitude));
            _terminalPwm.MinOnTime = 0.50;
            _terminalPwm.MinOffTime = TimeWarp.fixedDeltaTime;
            Core.Thrust.TargetThrottle = _terminalPwm.ThrottleCommand(desiredAcceleration, VesselState.MinThrustAcceleration,
                VesselState.MaxThrustAcceleration, TimeWarp.fixedDeltaTime);
        }

        private void TickVisualAssessment()
        {
            Core.Warp.MinimumWarp(true);
            Vector3d brakingError = TerminalVelocityError();
            Core.Attitude.attitudeTo(-brakingError, AttitudeReference.INERTIAL_COT, this);
            Core.Thrust.TargetThrottle = 1.0f;
            RefreshPreflight(false);
            if (Preflight?.Estimate == null || !Preflight.Estimate.HasImpact)
            {
                RejectController("V2 local visual assessment has no valid impact estimate.");
                return;
            }
            if (Preflight.Estimate.TargetError > VisualRebaseAccuracyLimit)
            {
                RejectController("V2 approach is outside the visual-rebase accuracy gate.");
                return;
            }
            MainBody.GetLatLngAltAtUT(Preflight.Estimate.ImpactUT, Preflight.Estimate.ImpactPosition, out double latitude, out double longitude, out _);
            SetActiveTarget(latitude, longitude, true);
            _visualRebaseDone = true;
            _pendingTargetEvent = "visual_rebase";
            _siteAssessment = AssessLocalSite(latitude, longitude);
            if (!_siteAssessment.Accepted)
            {
                RejectController("V2 local site assessment rejected the rebased target: " + _siteAssessment.Detail);
                return;
            }
            TransitionTo(V2FlightPhase.TerminalDivert, "V2 visual rebase complete; terminal-divert guidance is tracking the assessed local target.");
        }

        private Vector3d TerminalVelocityError()
        {
            Vector3d target = MainBody.GetWorldSurfacePosition(V2ActiveTargetLatitude, V2ActiveTargetLongitude,
                MainBody.TerrainAltitude(V2ActiveTargetLatitude, V2ActiveTargetLongitude, true));
            Vector3d horizontalError = Vector3d.Exclude(VesselState.Up, target - VesselState.CoM);
            double verticalSpeed = -Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up);
            double timeToGround = Math.Max(3.0, VesselState.AltitudeBottom / Math.Max(0.5, verticalSpeed));
            Vector3d desiredHorizontalVelocity = horizontalError.sqrMagnitude < 1.0
                ? Vector3d.zero
                : horizontalError.normalized * Math.Min(12.0, horizontalError.magnitude / timeToGround);
            Vector3d desiredVelocity = desiredHorizontalVelocity - Core.Hoverslam.FinalDescentSpeed * VesselState.Up;
            return VesselState.SurfaceVelocity - desiredVelocity;
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
            double reserve = _activePlan == null ? 20.0 : _activePlan.TerminalDivertReserve;
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

        private void SyncSelectedTarget()
        {
            if (!_hasActiveTarget || Math.Abs((double)Core.Target.targetLatitude - _selectedTargetLatitude) < 1e-8 && Math.Abs((double)Core.Target.targetLongitude - _selectedTargetLongitude) < 1e-8)
                return;
            _selectedTargetLatitude = (double)Core.Target.targetLatitude;
            _selectedTargetLongitude = (double)Core.Target.targetLongitude;
            SetActiveTarget(_selectedTargetLatitude, _selectedTargetLongitude, true);
            _siteAssessment = null;
            _pendingTargetEvent = "target_updated_by_player";
        }

        private void SetActiveTarget(double latitude, double longitude, bool traceEvent)
        {
            _activeTargetLatitude = latitude;
            _activeTargetLongitude = longitude;
            _hasActiveTarget = true;
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
            LandingGuidanceV2Estimate estimate = Preflight?.Estimate;
            if (estimate != null && estimate.HasImpact && estimate.SnapshotVersion == Preflight.Snapshot.Version)
            {
                MainBody.GetLatLngAltAtUT(estimate.ImpactUT, estimate.ImpactPosition, out double latitude, out double longitude, out _);
                GLUtils.DrawGroundMarker(MainBody, latitude, longitude, Color.blue, true, 0, MainBody.Radius / 14);
            }
        }

        private sealed class LandingSiteAssessment
        {
            public readonly bool Accepted; public readonly double Slope; public readonly double Roughness; public readonly string Detail;
            public LandingSiteAssessment(bool accepted, double slope, double roughness, string detail)
            { Accepted = accepted; Slope = slope; Roughness = roughness; Detail = detail; }
        }

        private void ReleaseV2Control()
        {
            Core.Thrust.ThrustOff();
            Core.Thrust.Users.Remove(this);
            Core.Attitude.Users.Remove(this);
            Core.GetComputerModule<MechJebModuleLandingPredictions>()?.Users.Remove(this);
        }
        private void BeginFiniteBurn(string name, double plannedDeltaV)
        {
            _phaseBurnName = name;
            _phaseBurnPlannedDeltaV = plannedDeltaV;
            _phaseBurnStartVelocity = VesselState.OrbitalVelocity;
        }

        private void TransitionTo(V2FlightPhase next, string status)
        {
            _flightPhase = next;
            ControllerStatus = status;
            if (StructuredTraceEnabled && HighLogic.LoadedSceneIsFlight && Core.Target.PositionTargetExists && Vessel != null)
                RefreshPreflight();
        }

        public void RefreshPreflight(bool forcePlan = false)
        {
            if (!HighLogic.LoadedSceneIsFlight || !Core.Target.PositionTargetExists || Vessel == null || MainBody == null)
            {
                Preflight = null;
                return;
            }

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
            if (forcePlan || Preflight?.AirlessPlan == null || VesselState.Time >= _nextPlanRefreshUT)
            {
                airlessPlan = AirlessLandingPlanner.Plan(snapshot);
                _nextPlanRefreshUT = VesselState.Time + PlanRefreshInterval;
            }
            else
            {
                airlessPlan = Preflight.AirlessPlan;
            }
            ReentrySimulation.Result atmosphericEstimate = Core.GetComputerModule<MechJebModuleLandingPredictions>()?.Result;
            AtmosphericLandingPlan atmosphericPlan = AtmosphericLandingPlanner.Plan(snapshot, atmosphericEstimate);
            Preflight = new LandingGuidanceV2Preflight(snapshot, estimate, estimatorValidation, assessment, airlessPlan,
                brakingLowerBound, atmosphericPlan);
            WriteCorrelatedTrace(Preflight);
        }

        private LandingGuidanceV2Snapshot CaptureSnapshot()
        {
            Core.StageStats.RequestUpdate();
            double availableDeltaV = Core.StageStats.VacStats.Sum(s => s.DeltaV);

            return new LandingGuidanceV2Snapshot(++_snapshotVersion, VesselState.Time, MainBody,
                VesselState.OrbitalPosition, VesselState.OrbitalVelocity, VesselState.Mass, availableDeltaV,
                VesselState.LimitedMaxThrustAcceleration, V2ActiveTargetLatitude, V2ActiveTargetLongitude,
                Vessel.LandedOrSplashed);
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
                string v1Phase = JsonString(phase);
                string v1Status = JsonString(status);
                string predOutcome = JsonString(predictionOutcome);
                string baseFields = string.Format(CultureInfo.InvariantCulture,
                    "\"snapshotVersion\":{0},\"ut\":{1},\"body\":\"{2}\",\"targetLat\":{3},\"targetLon\":{4}," +
                    "\"position\":[{5},{6},{7}],\"velocity\":[{8},{9},{10}],\"mass\":{11}," +
                    "\"availableDeltaV\":{12},\"maxAcceleration\":{13},\"outcome\":\"{14}\",\"impactUT\":{15}," +
                    "\"targetError\":{16},\"brakingDeltaVLowerBound\":{17},\"deltaVAboveLowerBound\":{18}," +
                    "\"v1Phase\":{19},\"v1Status\":{20},\"v1PredictionVersion\":{21},\"v1PredictionOutcome\":{22}," +
                    "\"v1PredictionEndLat\":{23},\"v1PredictionEndLon\":{24},\"v1PredictionEndUT\":{25},\"v1TargetError\":{26}," +
                    "\"warpRate\":{27},\"attitudeErrorDegrees\":{28},\"commandedThrottle\":{29},\"flightControlThrottle\":{30}," +
                    "\"actualThrustAcceleration\":{31},\"forwardThrustAcceleration\":{32},\"mechjebRcsEnabled\":{33}," +
                    "\"rcsActionGroupEnabled\":{34},\"rcsCommand\":[{35},{36},{37}],\"burnActive\":{38}," +
                    "\"isLandedOrSplashed\":{39},\"estimatorApplicable\":{40},\"flightDataValid\":{41},\"validityReason\":{42}," +
                    "\"estimatorRepeatOutcome\":{43},\"estimatorRepeatImpactUT\":{44},\"estimatorDeterministic\":{45},\"estimatorValidationDetail\":{46}," +
                    "\"preflightState\":{47},\"preflightLocalGravity\":{48},\"preflightReason\":{49}," +
                    "\"airlessPlanState\":{50},\"strategicDeorbitDeltaV\":{51},\"planTerminalLowerBound\":{52},\"planLowerBoundMargin\":{53}," +
                    "\"planDownrange\":{54},\"planCrossRange\":{55},\"planCorridorLimit\":{56},\"planReason\":{57},\"v2CommandAuthorized\":{58},\"v2Phase\":{59},\"v2Status\":{60},\"v2AutoWarp\":{61},\"planPlaneBurnUT\":{62},\"planStrategicBurnUT\":{63},\"planBrakingEntryUT\":{64}",
                    snapshot.Version, JsonNumber(snapshot.UT), EscapeJson(snapshot.Body.bodyName), JsonNumber(snapshot.TargetLatitude),
                    JsonNumber(snapshot.TargetLongitude), JsonNumber(snapshot.Position.x), JsonNumber(snapshot.Position.y),
                    JsonNumber(snapshot.Position.z), JsonNumber(snapshot.Velocity.x), JsonNumber(snapshot.Velocity.y),
                    JsonNumber(snapshot.Velocity.z), JsonNumber(snapshot.Mass), JsonNumber(snapshot.AvailableDeltaV),
                    JsonNumber(snapshot.MaximumAcceleration), estimate.Outcome, JsonNumber(estimate.ImpactUT), JsonNumber(estimate.TargetError),
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
                    ",\"atmosphericPlanState\":{0},\"atmosphericTargetError\":{1},\"atmosphericEntryCorridor\":{2},\"atmosphericEndpointUncertainty\":{3},\"atmosphericTerminalReserve\":{4},\"atmosphericLandingMargin\":{5},\"atmosphericPlanReason\":{6}",
                    JsonString(atmosphericPlan?.State.ToString()), JsonNumber(atmosphericPlan?.PredictedTargetError ?? double.NaN),
                    JsonNumber(atmosphericPlan?.EntryCorridorRadius ?? double.NaN), JsonNumber(atmosphericPlan?.EndpointUncertainty ?? double.NaN),
                    JsonNumber(atmosphericPlan?.TerminalReserve ?? double.NaN), JsonNumber(atmosphericPlan?.LandingMargin ?? double.NaN),
                    JsonString(atmosphericPlan?.Reason));
                baseFields += string.Format(CultureInfo.InvariantCulture,
                    ",\"v2ActiveTargetLat\":{0},\"v2ActiveTargetLon\":{1},\"v2VisualRebaseDone\":{2},\"v2SiteAccepted\":{3},\"v2SiteSlopeDegrees\":{4},\"v2SiteRoughness\":{5},\"v2SiteDetail\":{6},\"v2FiniteBurn\":{7},\"v2PlannedBurnDeltaV\":{8},\"v2DeliveredBurnDeltaV\":{9}",
                    JsonNumber(V2ActiveTargetLatitude), JsonNumber(V2ActiveTargetLongitude), _visualRebaseDone ? "true" : "false",
                    _siteAssessment != null && _siteAssessment.Accepted ? "true" : "false", JsonNumber(_siteAssessment?.Slope ?? double.NaN),
                    JsonNumber(_siteAssessment?.Roughness ?? double.NaN), JsonString(_siteAssessment?.Detail), JsonString(_phaseBurnName),
                    JsonNumber(_phaseBurnPlannedDeltaV), JsonNumber(_phaseBurnStartVelocity.sqrMagnitude > 0
                        ? (VesselState.OrbitalVelocity - _phaseBurnStartVelocity).magnitude : double.NaN));

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
