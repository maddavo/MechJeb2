using System;
using KSP.Localization;

namespace MuMech
{
    namespace Landing
    {
        public class CourseCorrection : AutopilotStep
        {
            private const double PulseCompletionDv = 0.05;
            private const double MinimumUsefulCorrectionDv = 0.05;
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
            private double _requestedPulseDv;
            private double _impactApprovedPulseDv;
            private double _impactSafetyRadius;
            private double _lastPulseStartedUT = double.NegativeInfinity;

            // Predictor snapshots taken during a pulse, or before post-burn
            // settling, cannot describe the orbit used by the next decision.
            internal bool PredictionSnapshotSafe(double inputUT) =>
                !_courseCorrectionBurning &&
                inputUT >= _lastPulseStartedUT &&
                (!_waitingForPostBurnPrediction ||
                 inputUT >= _postBurnCompletionUT + 0.75);

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
                        $"requestedPulseDv={_requestedPulseDv:F3} impactApprovedPulseDv={_impactApprovedPulseDv:F3} " +
                        $"impactSafetyRadius={_impactSafetyRadius:F1} pulseDir=({_pulseDirection.x:F4},{_pulseDirection.y:F4},{_pulseDirection.z:F4}) " +
                        $"burning={_courseCorrectionBurning} waiting={_waitingForPostBurnPrediction} " +
                        $"waitForPredictionVersion={_predictionVersionAtPulseCompletion} " +
                        $"pendingPredictions={_pendingPredictionCount}";
                }
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
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
                    if (!V1LandingControlPolicy.PostPulsePredictionSettled(
                            Core.Landing.PredictionVersion, _predictionVersionAtPulseCompletion,
                            Core.Landing.Prediction.InputUT, _postBurnCompletionUT))
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
                    if (!BeginPulse(_pendingCorrection, currentError))
                        return new CoastToDeceleration(Core);
                }
                else if (_remainingPulseDv <= 0)
                {
                    Vector3d deltaV = Core.Landing.ComputeCourseCorrection(true, DownrangeCaptureDistance,
                        MaximumDownrangeHandoffDistance);
                    if (deltaV.magnitude <= MinimumUsefulCorrectionDv)
                        return new CoastToDeceleration(Core);

                    if (!BeginPulse(deltaV, currentError))
                        return new CoastToDeceleration(Core);
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

            private double MaximumDownrangeHandoffDistance =>
                V1LandingControlPolicy.CourseCorrectionHandoffDistance(MainBody.Radius,
                    VesselState.SpeedSurface, VesselState.LimitedMaxThrustAcceleration);

            private bool BeginPulse(Vector3d deltaV, double targetError)
            {
                Vector3d direction = deltaV.normalized;
                double maximumPulseDv = V1LandingControlPolicy.MaximumCourseCorrectionPulse(
                    targetError, MainBody.Radius, _hasLastPulseDirection,
                    _hasLastPulseDirection ? Vector3d.Angle(_lastPulseDirection, direction) : 0);

                _requestedPulseDv = Math.Min(deltaV.magnitude, maximumPulseDv);
                double protectedDescentRadius = MainBody.Radius + Core.Landing.DecelerationEndAltitude() - 100;
                double impactMargin = Math.Max(100, MainBody.Radius * 0.0005);
                _impactSafetyRadius = protectedDescentRadius - impactMargin;
                _impactApprovedPulseDv = CourseCorrectionPulseSafety.LimitToImpactPreservingMagnitude(
                    _requestedPulseDv, MinimumUsefulCorrectionDv,
                    pulseDv => Orbit.PerturbedOrbit(VesselState.Time, pulseDv * direction).PeR < _impactSafetyRadius);
                _remainingPulseDv = _impactApprovedPulseDv;

                // No part of the requested correction retains an adequate
                // descent corridor.  Coast on the still-valid trajectory and
                // let the braking controller finish safely rather than lifting
                // periapsis and leaving the craft with no impact prediction.
                if (_remainingPulseDv <= 0)
                {
                    Core.Thrust.TargetThrottle = 0;
                    return false;
                }

                _pulseDirection = direction;
                _lastPulseStartedUT = VesselState.Time;
                _lastPulseDirection = direction;
                _hasLastPulseDirection = true;
                Status = Localizer.Format("#MechJeb_LandingGuidance_Status3",
                    deltaV.magnitude.ToString("F1")); //"Performing course correction of about " +  + " m/s"
                return true;
            }
        }
    }
}
