using System;
using System.Globalization;
using UnityEngine;
using KSP.Localization;

namespace MuMech
{
    namespace Landing
    {
        public class CourseCorrection : AutopilotStep
        {
            private const double MaxCorrectionPulseDv = 1.0;
            private const double PulseCompletionDv = 0.05;
            private const double MinimumUsefulCorrectionDv = 0.05;
            private const double PostBurnPredictionSettlingTime = 0.75;
            private const double CandidateDirectionAgreementAngle = 20;
            private const int RequiredCandidatePredictions = 2;

            private bool _courseCorrectionBurning;
            private bool _waitingForPostBurnPrediction;
            private double _remainingPulseDv;
            private Vector3d _pulseDirection;
            private long _predictionVersionAtPulseCompletion = -1;
            private double _postBurnCompletionUT;
            private bool _hasPendingCorrection;
            private Vector3d _pendingCorrection;
            private int _pendingPredictionCount;
            private bool _hasLastPulseDirection;
            private Vector3d _lastPulseDirection;
            private double _requestedCorrectionDv, _selectedPulseDv, _achievedPulseDv;
            private double _pulseStartUT = double.NaN, _pulseCutoffUT = double.NaN;
            private double _correctionCycleSeconds = double.NaN, _executableBrakeDeadlineUT = double.NaN;
            private string _deadlineSource = "unavailable", _handoffReason = "none", _interruptionReason = "none";
            private bool _deadlineHandoffPending, _responsePending, _measuringPreparation;
            private Vector3d _preparationDirection;
            private long _observedPredictionVersion = -1;
            private double _observedInputUT = double.NaN, _lastPublicationUT = double.NaN;
            private double _predictionRefreshSeconds = double.NaN, _lastDriveUT = double.NaN;
            private double _postBurnRefreshSeconds = double.NaN;
            private Vector3d _observedEndpoint, _coastEndpointRate, _pulseEndpoint, _pulseExpectedResponse;
            private double _pulseEndpointInputUT, _pulseReferenceUT, _trustedPulseDv = double.NaN;
            private string _pulseSelectionFailure = "none";
            private double _tailDv, _pulseMaximumThrust, _pulseThrottleLimit;
            private CorrectionBurnPlan _coarseBurn;
            private bool _coarseTried;
            private bool _compatibleResponse;
            private string _coarseReason = "not_requested";
            private double _observedReferenceUT, _observedTerrainASL, _pulseTerrainASL, _pulseDecelASL;
            private bool _coastRateReliable, _pulseCoastRateReliable;
            private Type _pulsePolicyType;
            private double _observedAvailableAcceleration = double.NaN;
            private object _contextBody;
            private double _contextLatitude = double.NaN, _contextLongitude = double.NaN;

            // Read-only predictor snapshot gate; does not change V1 decisions.
            internal bool PredictionSnapshotSafe(double inputUT) =>
                !_courseCorrectionBurning &&
                (!_waitingForPostBurnPrediction ||
                 inputUT >= _postBurnCompletionUT + PostBurnPredictionSettlingTime &&
                 (!Core.Landing.AirlessTargetedForecast ||
                  VesselState.CurrentThrustAcceleration == 0 && Vessel.ctrlState.mainThrottle == 0));

            // A completed pulse changes the physical orbit. Beta must not
            // consume its pre-pulse endpoint while awaiting a new prediction.
            // Keep the old result visible during the pulse itself so Beta can
            // finish that already-commanded burn.
            internal bool PublishedPredictionUsable(double inputUT) =>
                !_waitingForPostBurnPrediction ||
                inputUT >= _postBurnCompletionUT + PostBurnPredictionSettlingTime &&
                (!Core.Landing.AirlessTargetedForecast ||
                 VesselState.CurrentThrustAcceleration == 0 && Vessel.ctrlState.mainThrottle == 0);

            internal bool ForecastChanging => _remainingPulseDv > 0 || _waitingForPostBurnPrediction;
            internal string[] CorrectionInformation => new[]
            {
                "Correction estimate: " + LandingForecastInformation.DeltaV(_requestedCorrectionDv),
                "Selected burn: " + LandingForecastInformation.DeltaV(_selectedPulseDv),
                "Burn remaining: " + LandingForecastInformation.DeltaV(Math.Max(0, _remainingPulseDv - PulseCompletionDv)),
                "Delivered: " + LandingForecastInformation.DeltaV(_achievedPulseDv)
            };

            public CourseCorrection(MechJebCore core) : base(core)
            {
            }

            public override string TraceDetails
            {
                get
                {
                    double targetError = Core.Landing.PredictionReady
                        ? Vector3d.Distance(Core.Target.GetPositionTargetPosition(), Core.Landing.LandingSite)
                        : double.NaN;
                    double downrangeError = Core.Landing.LastCourseCorrectionDownrangeError;
                    double longBias = Core.Landing.LastCourseCorrectionLongBias;
                    return $" targetError={targetError:F1} downrangeError={downrangeError:F1} longBias={longBias:F1} " +
                        $"handoffLimit={Core.Landing.LastCourseCorrectionHandoffLimit:F1} pulseDv={_remainingPulseDv:F3} " +
                        $"pulseDir=({_pulseDirection.x:F4},{_pulseDirection.y:F4},{_pulseDirection.z:F4}) " +
                        $"burning={_courseCorrectionBurning} waiting={_waitingForPostBurnPrediction} " +
                        $"commandedPulseDv={(_selectedPulseDv > 0 ? V1LandingControlPolicy.CorrectionCommandForDelivery(_selectedPulseDv) : 0):R} measuredTailDv={_tailDv:R} " +
                        $"waitForPredictionVersion={_predictionVersionAtPulseCompletion} " +
                        $"pendingPredictions={_pendingPredictionCount}" + string.Format(CultureInfo.InvariantCulture,
                        " requestedCorrectionDv={0:R} selectedPulseDv={1:R} achievedPulseDv={2:R} pulseStartUT={3:R}" +
                        " pulseCutoffUT={4:R} postBurnReadyUT={5:R} correctionCycleSeconds={6:R}" +
                        " executableBrakeDeadlineUT={7:R} deadlineSource={8} handoffReason={9} interruptionReason={10}" +
                        " crossrangeError={11:R} measuringPreparation={12} preparationObserving={13} preparationResidualAngularSpeed={14:R}",
                        _requestedCorrectionDv, _selectedPulseDv, _achievedPulseDv, _pulseStartUT, _pulseCutoffUT,
                        Finite(_pulseCutoffUT) ? _postBurnCompletionUT + PostBurnPredictionSettlingTime : double.NaN,
                        _correctionCycleSeconds, _executableBrakeDeadlineUT, _deadlineSource, _handoffReason,
                        _interruptionReason, Core.Landing.LastCourseCorrectionCrossrangeError, _measuringPreparation,
                        Core.Landing.WarpPreparation.IsObserving, Core.Landing.WarpPreparation.ResidualAngularSpeed) +
                        " coarseReason=" + _coarseReason + string.Format(CultureInfo.InvariantCulture,
                            " coarseStartUT={0:R} coarseSourceUT={1:R} coarseBaselineError={2:R} coarseExpectedError={3:R}",
                            _coarseBurn?.BurnStartUT ?? double.NaN, _coarseBurn?.Initial.InputUT ?? double.NaN,
                            _coarseBurn?.BaselineError ?? double.NaN, _coarseBurn?.ExpectedError ?? double.NaN);
                }
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (UseAirlessPolicy()) return DriveAirless();
                if (!Core.Landing.PredictionReady)
                    return this;

                // If the atomospheric drag is at least 100mm/s2 then start trying to target the overshoot using the parachutes
                if (Core.Landing.DeployChutes)
                {
                    if (Core.Landing.ParachutesDeployable())
                    {
                        Core.Landing.ControlParachutes();
                    }
                }

                double currentError = Vector3d.Distance(Core.Target.GetPositionTargetPosition(), Core.Landing.LandingSite);

                if (currentError < CompletionError)
                {
                    Core.Thrust.TargetThrottle = 0;
                    if (Core.Landing.RCSAdjustment)
                        Core.RCS.Enabled = true;
                    return new CoastToDeceleration(Core);
                }

                // If we're off course, but already too low, skip the course correction
                if (VesselState.AltitudeASL < Core.Landing.DecelerationEndAltitude() + 5)
                {
                    return new DecelerationBurn(Core);
                }


                // If a parachute has already been deployed then we will not be able to control attitude anyway, so move back to the coast to deceleration step.
                if (VesselState.ParachuteDeployed)
                {
                    Core.Thrust.TargetThrottle = 0;
                    return new CoastToDeceleration(Core);
                }

                // We are not in .90 anymore. Turning while under drag is a bad idea
                if (VesselState.DragAcceleration > 0.1)
                {
                    return new CoastToDeceleration(Core);
                }

                if (_waitingForPostBurnPrediction)
                {
                    Core.Thrust.TargetThrottle = 0;

                    // Do not turn a prediction made during, or immediately after, a burn into
                    // the next command. A result version only says when a simulation completed;
                    // its input snapshot can still predate the engine cut-off.
                    if (Core.Landing.PredictionVersion <= _predictionVersionAtPulseCompletion ||
                        Core.Landing.Prediction.InputUT < _postBurnCompletionUT + PostBurnPredictionSettlingTime)
                        return this;

                    Vector3d candidateCorrection = Core.Landing.ComputeCourseCorrection(true, DownrangeCaptureDistance,
                        MaximumDownrangeHandoffDistance);
                    if (candidateCorrection.magnitude <= MinimumUsefulCorrectionDv)
                        return new CoastToDeceleration(Core);

                    _predictionVersionAtPulseCompletion = Core.Landing.PredictionVersion;
                    if (_hasPendingCorrection &&
                        Vector3d.Angle(_pendingCorrection, candidateCorrection) <= CandidateDirectionAgreementAngle)
                    {
                        _pendingCorrection = candidateCorrection;
                        _pendingPredictionCount++;
                    }
                    else
                    {
                        _pendingCorrection = candidateCorrection;
                        _pendingPredictionCount = 1;
                        _hasPendingCorrection = true;
                    }

                    // A single nonlinear prediction can legitimately choose the other
                    // branch of the finite-difference solution. Require a second,
                    // independent post-burn snapshot before aiming the vehicle at it.
                    if (_pendingPredictionCount < RequiredCandidatePredictions)
                        return this;

                    _waitingForPostBurnPrediction = false;
                    _courseCorrectionBurning = false;
                    _hasPendingCorrection = false;
                    BeginPulse(_pendingCorrection, currentError);
                }
                else if (_remainingPulseDv <= 0)
                {
                    Vector3d deltaV = Core.Landing.ComputeCourseCorrection(true, DownrangeCaptureDistance,
                        MaximumDownrangeHandoffDistance);
                    if (deltaV.magnitude <= MinimumUsefulCorrectionDv)
                        return new CoastToDeceleration(Core);

                    BeginPulse(deltaV, currentError);
                }

                Core.Attitude.attitudeTo(_pulseDirection, AttitudeReference.INERTIAL, Core.Landing);

                if (Core.Attitude.attitudeAngleFromTarget() < 2)
                    _courseCorrectionBurning = true;
                else if (Core.Attitude.attitudeAngleFromTarget() > 30)
                    _courseCorrectionBurning = false;

                if (_courseCorrectionBurning)
                {
                    const double TIME_CONSTANT = 0.5;
                    Core.Thrust.ThrustForDv(_remainingPulseDv, TIME_CONSTANT);
                    _remainingPulseDv -= VesselState.CurrentThrustAcceleration * TimeWarp.fixedDeltaTime;

                    if (_remainingPulseDv <= PulseCompletionDv)
                    {
                        Core.Thrust.TargetThrottle = 0;
                        _remainingPulseDv = 0;
                        _courseCorrectionBurning = false;
                        _waitingForPostBurnPrediction = true;
                        _predictionVersionAtPulseCompletion = Core.Landing.PredictionVersion;
                        _postBurnCompletionUT = VesselState.Time;
                        _hasPendingCorrection = false;
                        _pendingPredictionCount = 0;
                    }
                }
                else
                {
                    Core.Thrust.TargetThrottle = 0;
                }

                return this;
            }

            private double CompletionError => Math.Max(200, MainBody.Radius * 0.0005);

            private double DownrangeCaptureDistance => Math.Max(100, MainBody.Radius * 0.005);

            private double MaximumDownrangeHandoffDistance
            {
                get
                {
                    double availableAcceleration = Math.Max(0.1, VesselState.LimitedMaxThrustAcceleration);
                    double brakingDistance = VesselState.SpeedSurface * VesselState.SpeedSurface / (2 * availableAcceleration);
                    return Math.Max(DownrangeCaptureDistance, Math.Min(MainBody.Radius * 0.01, brakingDistance));
                }
            }

            private void BeginPulse(Vector3d deltaV, double targetError)
            {
                double maximumPulseDv = MaxCorrectionPulseDv;
                double nearTargetDistance = Math.Max(250, MainBody.Radius * 0.01);

                if (targetError < nearTargetDistance)
                    maximumPulseDv = 0.1;
                else if (targetError < 4 * nearTargetDistance)
                    maximumPulseDv = 0.25;

                Vector3d direction = deltaV.normalized;
                if (_hasLastPulseDirection && Vector3d.Angle(_lastPulseDirection, direction) > 90)
                    maximumPulseDv = Math.Min(maximumPulseDv, 0.1);

                _remainingPulseDv = Math.Min(deltaV.magnitude, maximumPulseDv);
                _pulseDirection = direction;
                _lastPulseDirection = direction;
                _hasLastPulseDirection = true;
                Status = Localizer.Format("#MechJeb_LandingGuidance_Status3",
                    deltaV.magnitude.ToString("F1")); //"Performing course correction of about " +  + " m/s"
            }

            private AutopilotStep DriveAirless()
            {
                if (!ReferenceEquals(_contextBody, MainBody) || _contextLatitude != Core.Target.targetLatitude ||
                    _contextLongitude != Core.Target.targetLongitude || VesselState.Time < _lastDriveUT)
                    ResetAirlessHistory();
                _lastDriveUT = VesselState.Time;
                if (_waitingForPostBurnPrediction &&
                    (VesselState.CurrentThrustAcceleration > 0 || Vessel.ctrlState.mainThrottle > 0))
                {
                    // Include real engine spool-down in achieved impulse. The
                    // existing settling interval starts after actual thrust,
                    // so its snapshot cannot still be a powered input.
                    double tail = VesselState.CurrentThrustAcceleration * TimeWarp.fixedDeltaTime;
                    _achievedPulseDv += tail;
                    _tailDv += tail;
                    _postBurnCompletionUT = VesselState.Time;
                }
                if (_measuringPreparation)
                {
                    Core.Thrust.TargetThrottle = 0;
                    Core.Attitude.attitudeTo(_preparationDirection, AttitudeReference.INERTIAL, Core.Landing);
                    Core.Landing.WarpPreparation.Observe(VesselState.Forward, _preparationDirection,
                        Core.vessel.angularVelocity.magnitude, VesselState.DeltaT, MuUtils.PhysicsRunning());
                }
                if (!Core.Landing.PredictionReady && _coarseBurn == null)
                {
                    Core.Thrust.TargetThrottle = 0;
                    if (_remainingPulseDv > 0) FinishAirlessPulse("forecast_unavailable");
                    if (Finite(_executableBrakeDeadlineUT) && VesselState.Time >= _executableBrakeDeadlineUT)
                        _handoffReason = "overdue_awaiting_fresh_forecast";
                    return this;
                }

                ReentrySimulation.Result forecast = Core.Landing.Prediction;
                double deadline = forecast == null ? _executableBrakeDeadlineUT : V1LandingControlPolicy.CorrectionBrakeDeadline(forecast.HasFirstRequiredThrust,
                    forecast.FirstRequiredThrustUT, forecast.BrakeReferenceUT(double.NaN));
                string deadlineSource = forecast != null && forecast.HasFirstRequiredThrust && Finite(forecast.FirstRequiredThrustUT) ?
                    "first_required_thrust" : "controller_reference_guard";
                if (!_waitingForPostBurnPrediction && _remainingPulseDv <= 0)
                {
                    _executableBrakeDeadlineUT = deadline;
                    _deadlineSource = deadlineSource;
                }
                else if (Finite(deadline) && (!Finite(_executableBrakeDeadlineUT) || deadline < _executableBrakeDeadlineUT))
                {
                    _executableBrakeDeadlineUT = deadline;
                    _deadlineSource = deadlineSource;
                }

                if (_remainingPulseDv > 0)
                {
                    if (_coarseBurn != null)
                    {
                        Core.Attitude.attitudeTo(_pulseDirection, AttitudeReference.INERTIAL, Core.Landing);
                        if (forecast != null && (forecast.Outcome == ReentrySimulation.Outcome.IMPACT ||
                            forecast.Outcome == ReentrySimulation.Outcome.LANDED &&
                            (forecast.InputDecelEndAltitudeASL != _coarseBurn.Initial.DecelEndASL ||
                             Math.Abs(forecast.BrakeReferenceUT(double.NaN) - _pulseReferenceUT) > VesselState.DeltaT)))
                        { FinishAirlessPulse("coarse_model_changed"); return this; }
                        if (Core.Thrust.LimiterMinThrottle || VesselState.MaxEngineResponseTime != 0 ||
                            Core.Thrust.ThrottleFixedLimit != _coarseBurn.Initial.MaximumCommandThrottle ||
                            !CorrectionBurnPlanner.CapabilityCompatible(VesselState.ThrustVectorMaxThrottle.magnitude,
                                _coarseBurn.Initial.MaximumThrust, VesselState.MaximumEngineMassFlow, _coarseBurn.Initial.MaximumMassFlow))
                        { FinishAirlessPulse("coarse_capability_changed"); return this; }
                        if (VesselState.Time < _coarseBurn.BurnStartUT)
                        { Core.Thrust.TargetThrottle = 0; return this; }
                        if (Core.Attitude.attitudeAngleFromTarget() >= 2 ||
                            !Finite(_pulseStartUT) && VesselState.Time - _coarseBurn.BurnStartUT > 2 * VesselState.DeltaT)
                        { _coarseBurn = null; FinishAirlessPulse("coarse_aim_deadline"); return this; }
                    }
                    if (VesselState.CurrentThrustAcceleration > 0 && Vessel.ctrlState.mainThrottle > 0)
                        _observedAvailableAcceleration = VesselState.CurrentThrustAcceleration / Vessel.ctrlState.mainThrottle;
                    double reserve = CycleReserve(_pulseDirection, !_courseCorrectionBurning);
                    double available = _executableBrakeDeadlineUT - VesselState.Time;
                    _correctionCycleSeconds = reserve + CommandPulseSeconds(_remainingPulseDv);
                    if (!Finite(_correctionCycleSeconds) || !Finite(available) || _correctionCycleSeconds >= available)
                    {
                        _deadlineHandoffPending = true;
                        FinishAirlessPulse("preparation_deadline");
                        // A real impulse invalidates the old forecast even when
                        // the interruption was needed for braking preparation.
                        if (_waitingForPostBurnPrediction) return this;
                        return Handoff(true, "preparation_deadline");
                    }
                    Core.Attitude.attitudeTo(_pulseDirection, AttitudeReference.INERTIAL, Core.Landing);
                    Core.Landing.WarpPreparation.Observe(VesselState.Forward, _pulseDirection,
                        Core.vessel.angularVelocity.magnitude, VesselState.DeltaT, MuUtils.PhysicsRunning());
                    if (Core.Attitude.attitudeAngleFromTarget() < 2) _courseCorrectionBurning = true;
                    else if (Core.Attitude.attitudeAngleFromTarget() > 30) _courseCorrectionBurning = false;
                    if (_courseCorrectionBurning)
                    {
                        if (!Finite(_pulseStartUT)) _pulseStartUT = VesselState.Time;
                        Core.Thrust.ThrustForDv(_remainingPulseDv, 0.5);
                        double delivered = VesselState.CurrentThrustAcceleration * TimeWarp.fixedDeltaTime;
                        _remainingPulseDv -= delivered; _achievedPulseDv += delivered;
                        if (_remainingPulseDv <= PulseCompletionDv) FinishAirlessPulse("completed", true);
                    }
                    else Core.Thrust.TargetThrottle = 0;
                    return this;
                }

                Core.Thrust.TargetThrottle = 0;
                if (_waitingForPostBurnPrediction && !V1LandingControlPolicy.PostPulsePredictionSettled(
                        Core.Landing.PredictionVersion, _predictionVersionAtPulseCompletion,
                        forecast.InputUT, _postBurnCompletionUT)) return this;

                // Do not use a completed publication twice, or two simulations
                // of the same input state as independent agreement.
                if (Core.Landing.PredictionVersion == _observedPredictionVersion ||
                    forecast.InputUT <= _observedInputUT) return this;
                Vector3d endpoint = Core.Landing.LandingSite - Core.Target.GetPositionTargetPosition();
                double currentError = endpoint.magnitude;
                _predictionRefreshSeconds = V1LandingControlPolicy.CorrectionRefreshSeconds(
                    VesselState.Time, forecast.InputUT, _lastPublicationUT,
                    _waitingForPostBurnPrediction ? _postBurnCompletionUT + PostBurnPredictionSettlingTime : double.NaN,
                    _waitingForPostBurnPrediction ? _postBurnRefreshSeconds : double.NaN);
                if (_waitingForPostBurnPrediction) _postBurnRefreshSeconds = _predictionRefreshSeconds;
                _lastPublicationUT = VesselState.Time;
                Vector3d correction = Core.Landing.ComputeCourseCorrection(true);
                _requestedCorrectionDv = correction.magnitude;
                double directionChange = _hasLastPulseDirection ? Vector3d.Angle(_lastPulseDirection, correction) : 0;
                if (_responsePending)
                {
                    Vector3d observedResponse = endpoint - _pulseEndpoint -
                        _coastEndpointRate * (forecast.InputUT - _pulseEndpointInputUT);
                    double disagreement = (observedResponse - _pulseExpectedResponse * _achievedPulseDv).magnitude;
                    bool sameModel = _pulseCoastRateReliable &&
                        Math.Abs(forecast.BrakeReferenceUT(double.NaN) - _pulseReferenceUT) <= TimeWarp.fixedDeltaTime &&
                        forecast.InputProbableLandingSiteASL == _pulseTerrainASL &&
                        forecast.InputDecelEndAltitudeASL == _pulseDecelASL &&
                        forecast.InputDescentSpeedPolicy?.GetType() == _pulsePolicyType;
                    _trustedPulseDv = V1LandingControlPolicy.CorrectionResponseTrust(
                        _selectedPulseDv, _achievedPulseDv, disagreement, CompletionError / 2,
                        sameModel, directionChange);
                    _compatibleResponse = V1LandingControlPolicy.SingleCorrectionForecastSufficient(sameModel,
                        disagreement, CompletionError / 2, directionChange,
                        VesselState.ThrustVectorMaxThrottle.magnitude >= .9 * _pulseMaximumThrust &&
                        Core.Thrust.ThrottleFixedLimit == _pulseThrottleLimit && !Core.Thrust.LimiterMinThrottle);
                    _responsePending = false;
                }
                else if (Finite(_observedInputUT) && !_waitingForPostBurnPrediction)
                {
                    _coastRateReliable = Math.Abs(forecast.BrakeReferenceUT(double.NaN) - _observedReferenceUT) <=
                        TimeWarp.fixedDeltaTime && forecast.InputProbableLandingSiteASL == _observedTerrainASL;
                    _coastEndpointRate = _coastRateReliable ?
                        (endpoint - _observedEndpoint) / (forecast.InputUT - _observedInputUT) : Vector3d.zero;
                }
                _observedEndpoint = endpoint; _observedInputUT = forecast.InputUT;
                _observedPredictionVersion = Core.Landing.PredictionVersion;
                _observedReferenceUT = forecast.BrakeReferenceUT(double.NaN);
                _observedTerrainASL = forecast.InputProbableLandingSiteASL;

                double handoffReserve = CycleReserve(correction.normalized, false);
                if (_deadlineHandoffPending ||
                    !Finite(_executableBrakeDeadlineUT) ||
                    (Finite(handoffReserve) && VesselState.Time + handoffReserve >= _executableBrakeDeadlineUT))
                    return Handoff(true, "preparation_deadline");
                if (currentError < CompletionError) return Handoff(false, "target_converged");
                if (VesselState.AltitudeASL < Core.Landing.DecelerationEndAltitude() + 5)
                    return Handoff(true, "terminal_altitude");
                // A small/degenerate solver demand cannot certify accuracy while
                // the independently measured endpoint remains outside capture.
                if (!Finite(correction.magnitude) || correction.magnitude <= 0)
                    return Handoff(false, "no_useful_correction");

                if (_hasPendingCorrection && Vector3d.Angle(_pendingCorrection, correction) <= CandidateDirectionAgreementAngle)
                    _pendingPredictionCount++;
                else { _pendingPredictionCount = 1; _hasPendingCorrection = true; }
                _pendingCorrection = correction;
                if (_pendingPredictionCount < (_compatibleResponse ? 1 : RequiredCandidatePredictions)) return this;
                _waitingForPostBurnPrediction = false;
                if (!Finite(CycleReserve(correction.normalized, true)))
                {
                    // Bootstrap a passive observation using the same unpowered
                    // aim as V1 already uses. No pulse can begin with unknown
                    // turn/settling time. A measurement already in progress
                    // must finish settling after crossing the alignment gate.
                    // The validated thrust deadline remains authoritative; an
                    // earlier reference release is not ignition.
                    double unmeasuredDeadline = _executableBrakeDeadlineUT;
                    if (!Finite(Core.Landing.WarpPreparation.Estimate(0)) &&
                        V1LandingControlPolicy.CorrectionPreparationUnavailable(
                            Core.Landing.WarpPreparation.IsObserving,
                            Vector3d.Angle(VesselState.Forward, correction)))
                        return Handoff(true, "preparation_measurement_unavailable");
                    if (VesselState.Time + 0.75 + 2 * _predictionRefreshSeconds + TimeWarp.fixedDeltaTime >= unmeasuredDeadline)
                        return Handoff(true, "preparation_measurement_unavailable");
                    if (!_measuringPreparation)
                    {
                        _preparationDirection = correction.normalized; _measuringPreparation = true;
                        Core.Attitude.attitudeTo(_preparationDirection, AttitudeReference.INERTIAL, Core.Landing);
                        Core.Landing.WarpPreparation.Observe(VesselState.Forward, _preparationDirection,
                            Core.vessel.angularVelocity.magnitude, VesselState.DeltaT, MuUtils.PhysicsRunning());
                    }
                    return this;
                }
                _measuringPreparation = false;
                if (!BeginAirlessPulse(correction, endpoint, directionChange))
                    return Handoff(true, _pulseSelectionFailure);
                return this;
            }

            private double PulseSeconds(double deliveredDv) => CommandPulseSeconds(
                V1LandingControlPolicy.CorrectionCommandForDelivery(deliveredDv));
            private double CommandPulseSeconds(double dv) => V1LandingControlPolicy.CorrectionPulseSeconds(dv,
                Finite(_observedAvailableAcceleration) ?
                    Math.Min(VesselState.LimitedMaxThrustAcceleration, _observedAvailableAcceleration) :
                    VesselState.LimitedMaxThrustAcceleration, VesselState.MaxThrustAcceleration,
                VesselState.MaxEngineResponseTime, Core.Thrust.LimiterMinThrottle ? (double)Core.Thrust.MinThrottle : 0,
                VesselState.ThrustMinimum / Math.Max(VesselState.Mass, 1e-9), TimeWarp.fixedDeltaTime);

            private bool UseAirlessPolicy()
            {
                if (Core.Landing.AirlessTargetedForecast) return true;
                if (_contextBody == null) return false;
                ResetAirlessHistory();
                _contextBody = null;
                Core.Attitude.attitudeDeactivate();
                return false;
            }

            private void ResetAirlessHistory()
            {
                bool realPulse = V1LandingControlPolicy.CorrectionResetRequiresSettling(
                    _waitingForPostBurnPrediction, VesselState.CurrentThrustAcceleration,
                    Vessel.ctrlState.mainThrottle, _remainingPulseDv > 0 ? _achievedPulseDv : 0);
                Core.Thrust.TargetThrottle = 0;
                _remainingPulseDv = 0; _courseCorrectionBurning = false;
                _waitingForPostBurnPrediction = realPulse; _hasPendingCorrection = false;
                _pendingPredictionCount = 0; _hasLastPulseDirection = false;
                _deadlineHandoffPending = false; _responsePending = false;
                _measuringPreparation = false;
                _coarseBurn = null; _coarseTried = false; _compatibleResponse = false;
                _trustedPulseDv = double.NaN; _executableBrakeDeadlineUT = double.NaN;
                _observedAvailableAcceleration = double.NaN;
                _predictionRefreshSeconds = double.NaN; _observedInputUT = double.NaN;
                _postBurnRefreshSeconds = double.NaN;
                _coastRateReliable = false; _pulseCoastRateReliable = false;
                _observedPredictionVersion = -1; _lastPublicationUT = double.NaN;
                _contextBody = MainBody; _contextLatitude = Core.Target.targetLatitude;
                _contextLongitude = Core.Target.targetLongitude;
                _requestedCorrectionDv = 0; _selectedPulseDv = 0; _achievedPulseDv = 0;
                _tailDv = 0;
                _pulseStartUT = double.NaN; _pulseCutoffUT = double.NaN;
                _correctionCycleSeconds = double.NaN; _deadlineSource = "unavailable";
                _handoffReason = "none"; _interruptionReason = "context_reset";
                _predictionVersionAtPulseCompletion = -1;
                if (!realPulse) return;
                _pulseCutoffUT = _postBurnCompletionUT = VesselState.Time;
                _predictionVersionAtPulseCompletion = Core.Landing.PredictionVersion;
                Core.Landing.InvalidateCourseCorrectionPrediction();
            }

            private double CycleReserve(Vector3d correctionDirection, bool includeAim)
            {
                double brakeUT = _executableBrakeDeadlineUT;
                if (!Finite(brakeUT)) return double.PositiveInfinity;
                Vector3d brakeDirection = -Orbit.WorldOrbitalVelocityAtUT(brakeUT) +
                    MainBody.getRFrmVel(Orbit.WorldPositionAtUT(brakeUT));
                double brakeAngle = Vector3d.Angle(includeAim ? correctionDirection : VesselState.Forward, brakeDirection);
                V1WarpPreparation timing = Core.Landing.WarpPreparation;
                double brakePreparation = timing.Estimate(brakeAngle);
                double aim = includeAim ? timing.Estimate(Vector3d.Angle(VesselState.Forward, correctionDirection)) : 0;
                return V1LandingControlPolicy.CorrectionCycleReserve(aim, _predictionRefreshSeconds,
                    brakePreparation, timing.WarpOverrunSeconds, TimeWarp.fixedDeltaTime);
            }

            private bool BeginAirlessPulse(Vector3d correction, Vector3d endpoint, double directionChange)
            {
                _pulseSelectionFailure = "none";
                _requestedCorrectionDv = correction.magnitude;
                Vector3d direction = correction.normalized;
                double localResponse = Core.Landing.LastCourseCorrectionLocalResponse.magnitude;
                double resolution = V1LandingControlPolicy.CorrectionDeliveryResolution(
                    VesselState.MaxThrustAcceleration, VesselState.ThrustMinimum / Math.Max(VesselState.Mass,1e-9),
                    Core.Thrust.LimiterMinThrottle ? (double)Core.Thrust.MinThrottle : 0,
                    TimeWarp.fixedDeltaTime, _tailDv);
                double maximum = Math.Min(correction.magnitude,
                    double.IsNaN(_trustedPulseDv) ? correction.magnitude : _trustedPulseDv);
                double movementLimit = correction.magnitude / 2 < resolution ? endpoint.magnitude : endpoint.magnitude / 2;
                // Reuse the descending-approach screen. The additional direct
                // orbit response check bounds extrapolation of the 1 m/s basis.
                double trust = CourseCorrectionPulseSafety.LimitToImpactPreservingMagnitude(maximum,
                    resolution, dv =>
                    {
                        try { return Core.Landing.CourseCorrectionGeometryTrusted(direction, dv,
                            movementLimit, Math.Min(CompletionError / 2,
                                Math.Max(1, localResponse * dv / 2))); }
                        catch (Exception) { return false; }
                    });
                double reserve = CycleReserve(direction, true);
                _selectedPulseDv = V1LandingControlPolicy.SelectCorrectionDelivery(correction.magnitude,
                    endpoint.magnitude, CompletionError, localResponse, trust, directionChange,
                    _executableBrakeDeadlineUT - VesselState.Time, reserve, resolution, PulseSeconds,
                    out _pulseSelectionFailure);
                if (_selectedPulseDv > 0 && (!_coarseTried || _coarseReason == "coarse_searching" || _coarseReason == "coarse_validating"))
                {
                    int prepared = Core.Landing.PrepareCorrectionBurn(direction, correction.magnitude,
                        _selectedPulseDv, Core.Landing.WarpPreparation.Estimate(Vector3d.Angle(VesselState.Forward, direction)),
                        _executableBrakeDeadlineUT - reserve, out var coarse, out _coarseReason);
                    if (prepared == 1)
                    {
                        _preparationDirection = direction; _measuringPreparation = true;
                        Status = "Validating larger correction burn";
                        return true;
                    }
                    _coarseTried = true;
                    if (prepared == 2)
                    {
                        _coarseBurn = coarse; _selectedPulseDv = coarse.Delivery; direction = coarse.Direction;
                        if (coarse.Forecast.FirstRequiredThrust.HasValue)
                            _executableBrakeDeadlineUT = Math.Min(_executableBrakeDeadlineUT, coarse.Forecast.FirstRequiredThrust.Value.UT);
                        _measuringPreparation = false;
                    }
                }
                _correctionCycleSeconds = reserve + PulseSeconds(_selectedPulseDv);
                if (_selectedPulseDv <= 0) return false;
                _remainingPulseDv = V1LandingControlPolicy.CorrectionCommandForDelivery(_selectedPulseDv);
                _pulseMaximumThrust = VesselState.ThrustVectorMaxThrottle.magnitude;
                _pulseThrottleLimit = Core.Thrust.ThrottleFixedLimit;
                _tailDv = 0;
                _pulseDirection = direction;
                _lastPulseDirection = direction; _hasLastPulseDirection = true;
                _achievedPulseDv = 0; _pulseStartUT = double.NaN; _pulseCutoffUT = double.NaN;
                _pulseEndpoint = endpoint; _pulseEndpointInputUT = Core.Landing.Prediction.InputUT;
                _pulseReferenceUT = Core.Landing.Prediction.BrakeReferenceUT(double.NaN);
                _pulseTerrainASL = Core.Landing.Prediction.InputProbableLandingSiteASL;
                _pulseDecelASL = Core.Landing.Prediction.InputDecelEndAltitudeASL;
                _pulsePolicyType = Core.Landing.Prediction.InputDescentSpeedPolicy?.GetType();
                _pulseCoastRateReliable = _coastRateReliable;
                // The solver compensates the endpoint at contact UT; response
                // measurements compare the forecast's current body-fixed site.
                _pulseExpectedResponse = Quaternion.AngleAxis(
                    (float)(-360 * (Core.Landing.Prediction.EndUT - VesselState.Time) / MainBody.rotationPeriod),
                    MainBody.angularVelocity.normalized) * Core.Landing.LastCourseCorrectionLocalResponse;
                _hasPendingCorrection = false; _pendingPredictionCount = 0; _interruptionReason = "none";
                Status = Localizer.Format("#MechJeb_LandingGuidance_Status3", correction.magnitude.ToString("F1"));
                return true;
            }

            private void FinishAirlessPulse(string reason, bool deliveredThisTick = false)
            {
                if (!deliveredThisTick)
                    _achievedPulseDv += VesselState.CurrentThrustAcceleration * TimeWarp.fixedDeltaTime;
                Core.Thrust.TargetThrottle = 0;
                _remainingPulseDv = 0; _courseCorrectionBurning = false;
                _interruptionReason = reason; _pulseCutoffUT = VesselState.Time;
                _coarseBurn = null; _coarseTried = reason.StartsWith("coarse_", StringComparison.Ordinal) && _achievedPulseDv == 0;
                _hasPendingCorrection = false; _pendingPredictionCount = 0;
                if (reason != "completed") Core.Attitude.attitudeDeactivate();
                // An applied engine command can still produce thrust after
                // cutoff even when spool-up has not delivered measured dv yet.
                if (!V1LandingControlPolicy.CorrectionResetRequiresSettling(false,
                        VesselState.CurrentThrustAcceleration, Vessel.ctrlState.mainThrottle, _achievedPulseDv)) return;
                _waitingForPostBurnPrediction = true; _responsePending = true;
                _predictionVersionAtPulseCompletion = Core.Landing.PredictionVersion;
                _postBurnCompletionUT = VesselState.Time;
                _lastPublicationUT = double.NaN; _postBurnRefreshSeconds = double.NaN;
                Core.Landing.InvalidateCourseCorrectionPrediction();
            }

            private AutopilotStep Handoff(bool deadline, string reason)
            {
                Core.Thrust.TargetThrottle = 0;
                _courseCorrectionBurning = false; _remainingPulseDv = 0;
                _measuringPreparation = false;
                Core.Attitude.attitudeDeactivate();
                Core.Warp.MinimumWarp();
                _handoffReason = reason;
                if (Core.Landing.RCSAdjustment) Core.RCS.Enabled = !deadline;
                Core.Landing.TraceCourseCorrectionHandoff(TraceDetails,
                    deadline ? nameof(DecelerationBurn) : nameof(CoastToDeceleration));
                return deadline ? (AutopilotStep)new DecelerationBurn(Core) : new CoastToDeceleration(Core);
            }

            private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
