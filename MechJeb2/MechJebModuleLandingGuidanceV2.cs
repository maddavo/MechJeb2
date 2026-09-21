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
        private double _nextRefreshUT;
        private long _snapshotVersion;
        private long _v1PredictionVersion;
        private ReentrySimulation.Result _lastV1Prediction;
        private string _lastV1Phase;
        private bool? _lastV1Burning;
        private bool? _lastWarped;
        private V2FlightPhase _flightPhase;
        private AirlessLandingPlan _activePlan;
        private Vector3d _burnTargetVelocity;
        private Vector3d _lastAdjustedVelocity;
        private string _lastV2Phase;
        private readonly DeltaSigmaThrottleModulator _terminalPwm = new DeltaSigmaThrottleModulator(0.02, 0.50);

        public enum V2FlightPhase { Idle, Preflight, WarpToStrategic, AlignStrategicBurn, StrategicBurn, Coast, BrakingApproach, TerminalDescent, Complete, Rejected }

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool PreviewEnabled;

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool StructuredTraceEnabled;

        [UsedImplicitly, Persistent(pass = (int)(Pass.GLOBAL | Pass.LOCAL))]
        public bool V2AutoWarp = true;

        public LandingGuidanceV2Preflight Preflight { get; private set; }

        public bool IsPreviewOnly => _flightPhase == V2FlightPhase.Idle || _flightPhase == V2FlightPhase.Rejected || _flightPhase == V2FlightPhase.Complete;
        public bool ControllerActive => _flightPhase == V2FlightPhase.Preflight || _flightPhase == V2FlightPhase.WarpToStrategic || _flightPhase == V2FlightPhase.AlignStrategicBurn || _flightPhase == V2FlightPhase.StrategicBurn || _flightPhase == V2FlightPhase.Coast || _flightPhase == V2FlightPhase.BrakingApproach || _flightPhase == V2FlightPhase.TerminalDescent;
        public V2FlightPhase FlightPhase => _flightPhase;
        public string ControllerStatus { get; private set; } = "Idle";

        public MechJebModuleLandingGuidanceV2(MechJebCore core) : base(core)
        {
            Enabled = true;
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

        public bool StartAirlessLanding()
        {
            if (!HighLogic.LoadedSceneIsFlight || !Core.Target.PositionTargetExists)
                return RejectController("Select a landing target before starting V2.");
            RefreshPreflight();
            if (Preflight?.AirlessPlan == null || Preflight.AirlessPlan.State != AirlessLandingPlanState.Candidate) return RejectController("V2 requires a current airless strategic-deorbit candidate.");
            _activePlan = Preflight.AirlessPlan;
            Core.Thrust.Users.Add(this); Core.Attitude.Users.Add(this);
            TransitionTo(V2FlightPhase.WarpToStrategic, "V2 plan accepted; moving to the strategic-deorbit burn gate.");
            return true;
        }

        public void AbortAirlessLanding()
        {
            ReleaseV2Control(); TransitionTo(V2FlightPhase.Idle, "V2 landing aborted.");
        }

        private bool RejectController(string reason) { ReleaseV2Control(); TransitionTo(V2FlightPhase.Rejected, reason); return false; }

        private void TickController()
        {
            if (Vessel == null || Vessel.LandedOrSplashed) { ReleaseV2Control(); TransitionTo(V2FlightPhase.Complete, "V2 landing completed: vessel is landed or splashed."); return; }
            if (Core.Landing != null && Core.Landing.Enabled) { RejectController("V1 Landing Guidance was engaged; V2 relinquished control."); return; }
            switch (_flightPhase)
            {
                case V2FlightPhase.WarpToStrategic:
                    Core.Thrust.ThrustOff();
                    if (VesselState.Time < _activePlan.StrategicBurnUT - 20.0 && V2AutoWarp)
                    {
                        Core.Warp.WarpToUT(_activePlan.StrategicBurnUT - 20.0);
                        break;
                    }
                    Core.Warp.MinimumWarp(true);
                    RefreshPreflight();
                    if (Preflight?.AirlessPlan == null || Preflight.AirlessPlan.State != AirlessLandingPlanState.Candidate)
                    {
                        RejectController("V2 plan failed fresh validation at the strategic-burn gate.");
                        break;
                    }
                    _activePlan = Preflight.AirlessPlan;
                    if (_activePlan.StrategicBurnUT > VesselState.Time + 25.0)
                        break;
                    _burnTargetVelocity = VesselState.OrbitalVelocity + _activePlan.StrategicDeorbitDeltaV;
                    TransitionTo(V2FlightPhase.AlignStrategicBurn, "Aligning for the validated V2 strategic deorbit burn.");
                    break;
                case V2FlightPhase.AlignStrategicBurn:
                    Core.Thrust.ThrustOff(); Core.Attitude.attitudeTo(_activePlan.StrategicDeorbitDeltaV, AttitudeReference.INERTIAL_COT, this);
                    if (Core.Attitude.attitudeError < 2.0) TransitionTo(V2FlightPhase.StrategicBurn, "Executing V2 strategic deorbit burn.");
                    break;
                case V2FlightPhase.StrategicBurn:
                    Core.Attitude.attitudeTo(_activePlan.StrategicDeorbitDeltaV, AttitudeReference.INERTIAL_COT, this);
                    double remainingDv = Vector3d.Dot(_burnTargetVelocity - VesselState.OrbitalVelocity, _activePlan.StrategicDeorbitDeltaV.normalized);
                    if (remainingDv <= 0.5)
                    {
                        Core.Thrust.ThrustOff();
                        TransitionTo(V2FlightPhase.Coast, "Strategic deorbit complete; coasting to V2 braking approach.");
                    }
                    else Core.Thrust.ThrustForDv(remainingDv, 0.5);
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
                case V2FlightPhase.BrakingApproach:
                    Vector3d adjustedVelocity = VesselState.SurfaceVelocity + Core.Hoverslam.FinalDescentSpeed * VesselState.Up;
                    Core.Attitude.attitudeTo(-adjustedVelocity, AttitudeReference.INERTIAL_COT, this);
                    Core.Thrust.TargetThrottle = 1.0f;
                    if (!double.IsNaN(_lastAdjustedVelocity.x) && Vector3d.Angle(_lastAdjustedVelocity, adjustedVelocity) > 10.0)
                    {
                        _terminalPwm.Reset();
                        TransitionTo(V2FlightPhase.TerminalDescent, "V2 terminal descent is controlling velocity and touchdown.");
                    }
                    _lastAdjustedVelocity = adjustedVelocity;
                    break;
                case V2FlightPhase.TerminalDescent:
                    TickTerminalDescent();
                    break;
            }
        }

        private void TickTerminalDescent()
        {
            if (Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up) >= -1.0)
                Core.Attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, this);
            else
                Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, this);
            double altitude = Math.Max(0.1, VesselState.AltitudeBottom);
            double acceleration = Vessel.graviticAcceleration.magnitude + 0.5 * (VesselState.SurfaceVelocity.sqrMagnitude - 0.25) / altitude;
            _terminalPwm.MinOnTime = 0.50;
            _terminalPwm.MinOffTime = TimeWarp.fixedDeltaTime;
            Core.Thrust.TargetThrottle = _terminalPwm.ThrottleCommand(acceleration, VesselState.MinThrustAcceleration, VesselState.MaxThrustAcceleration, TimeWarp.fixedDeltaTime);
        }

        private void ReleaseV2Control() { Core.Thrust.ThrustOff(); Core.Thrust.Users.Remove(this); Core.Attitude.Users.Remove(this); }
        private void TransitionTo(V2FlightPhase next, string status) { _flightPhase = next; ControllerStatus = status; }

        public void RefreshPreflight()
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
            AirlessLandingPlan airlessPlan = AirlessLandingPlanner.Plan(snapshot);
            Preflight = new LandingGuidanceV2Preflight(snapshot, estimate, estimatorValidation, assessment, airlessPlan,
                brakingLowerBound);
            WriteCorrelatedTrace(Preflight);
        }

        private LandingGuidanceV2Snapshot CaptureSnapshot()
        {
            Core.StageStats.RequestUpdate();
            double availableDeltaV = Core.StageStats.VacStats.Sum(s => s.DeltaV);

            return new LandingGuidanceV2Snapshot(++_snapshotVersion, VesselState.Time, MainBody,
                VesselState.OrbitalPosition, VesselState.OrbitalVelocity, VesselState.Mass, availableDeltaV,
                VesselState.LimitedMaxThrustAcceleration, Core.Target.targetLatitude, Core.Target.targetLongitude,
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
                    "\"planDownrange\":{54},\"planCrossRange\":{55},\"planCorridorLimit\":{56},\"planReason\":{57},\"v2CommandAuthorized\":{58},\"v2Phase\":{59},\"v2Status\":{60},\"v2AutoWarp\":{61}",
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
                    ControllerActive ? "true" : "false", JsonString(_flightPhase.ToString()), JsonString(ControllerStatus), V2AutoWarp ? "true" : "false");

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
