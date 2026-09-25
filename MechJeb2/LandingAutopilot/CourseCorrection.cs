using System;
using KSP.Localization;

namespace MuMech
{
    namespace Landing
    {
        public class CourseCorrection : AutopilotStep
        {
            private const double MinimumUsefulCorrectionDv = 0.05;
            private const double PostBurnPredictionSettlingTime = 0.75;
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
            private double _remoteEffectFraction = CourseCorrectionPulsePolicy.InitialRemoteEffectFraction;
            private int _verifiedRemoteResponses;
            private bool _lastPulseWasRemote;
            private bool _lastPulseReversed;
            private double _lastPulseTargetError;
            private double _lastPulseEffectFraction;

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
                        $"effectGain={_lastPulseEffectFraction:F3} remoteGain={_remoteEffectFraction:F3} " +
                        $"pulseDir=({_pulseDirection.x:F4},{_pulseDirection.y:F4},{_pulseDirection.z:F4}) " +
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
                    if (Core.Landing.PredictionVersion <= _predictionVersionAtPulseCompletion ||
                        Core.Landing.Prediction.InputUT < _postBurnCompletionUT + PostBurnPredictionSettlingTime)
                        return this;

                    Vector3d candidateCorrection = Core.Landing.ComputeCourseCorrection(true, DownrangeCaptureDistance,
                        MaximumDownrangeHandoffDistance);
                    if (candidateCorrection.magnitude <= MinimumUsefulCorrectionDv)
                        return new CoastToDeceleration(Core);

                    _predictionVersionAtPulseCompletion = Core.Landing.PredictionVersion;
                    if (_hasPendingCorrection)
                    {
                        _pendingCorrection = CourseCorrectionPredictionConsensus.AddSample(_pendingCorrection,
                            _pendingPredictionCount, candidateCorrection);
                        _pendingPredictionCount++;
                    }
                    else
                    {
                        _pendingCorrection = candidateCorrection;
                        _pendingPredictionCount = 1;
                        _hasPendingCorrection = true;
                    }

                    // Require two independent post-burn snapshots. Their finite-difference
                    // solutions can vary in direction even when the consensus predictor is
                    // stable; use their mean rather than resetting this wait forever.
                    if (_pendingPredictionCount < RequiredCandidatePredictions)
                        return this;

                    if (!CourseCorrectionPredictionConsensus.IsUsable(_pendingCorrection,
                            MinimumUsefulCorrectionDv))
                        return new CoastToDeceleration(Core);

                    UpdateRemoteEffectGain(currentError);
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

                    if (CourseCorrectionPulseExecutionPolicy.HasCompleted(_remainingPulseDv))
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
                double nearTargetDistance = Math.Max(250, MainBody.Radius * 0.01);
                Vector3d direction = deltaV.normalized;
                bool reversesPreviousPulse = _hasLastPulseDirection &&
                    Vector3d.Angle(_lastPulseDirection, direction) > 90;

                _lastPulseWasRemote = targetError >= 10 * nearTargetDistance;
                _lastPulseReversed = reversesPreviousPulse;
                _lastPulseTargetError = targetError;
                _lastPulseEffectFraction = CourseCorrectionPulsePolicy.SelectEffectFraction(
                    targetError, nearTargetDistance, reversesPreviousPulse, _remoteEffectFraction);
                _remainingPulseDv = deltaV.magnitude * _lastPulseEffectFraction;
                _pulseDirection = direction;
                _lastPulseDirection = direction;
                _hasLastPulseDirection = true;
                Status = Localizer.Format("#MechJeb_LandingGuidance_Status3",
                    deltaV.magnitude.ToString("F1")); //"Performing course correction of about " +  + " m/s"
            }

            private void UpdateRemoteEffectGain(double currentError)
            {
                if (!_lastPulseWasRemote || _lastPulseReversed || _lastPulseEffectFraction <= 0 ||
                    _lastPulseTargetError <= 0 || double.IsNaN(currentError) || double.IsInfinity(currentError))
                    return;

                double observedFraction = 1 - currentError / _lastPulseTargetError;
                _remoteEffectFraction = CourseCorrectionPulsePolicy.UpdateRemoteEffectFraction(
                    _remoteEffectFraction, _lastPulseEffectFraction, observedFraction,
                    ref _verifiedRemoteResponses);
            }
        }

        /// <summary>
        /// Chooses the fraction of the predictor's requested impact movement
        /// to apply in this pulse. The correction solver has already converted
        /// the target miss into a delta-V vector; scaling the vector scales its
        /// predicted surface effect without imposing a body-specific delta-V cap.
        /// </summary>
        public static class CourseCorrectionPulsePolicy
        {
            public const double InitialRemoteEffectFraction = 0.50;
            public const double MaximumRemoteEffectFraction = 0.80;
            private const double MinimumRemoteEffectFraction = 0.25;
            private const double RemoteEffectFractionStep = 0.10;

            public static double SelectEffectFraction(double targetError, double nearTargetDistance,
                bool reversesPreviousPulse, double remoteEffectFraction)
            {
                if (!IsFinite(targetError) || !IsFinite(nearTargetDistance) ||
                    targetError <= 0 || nearTargetDistance <= 0)
                    return 0;

                double fraction;
                if (targetError < nearTargetDistance)
                    fraction = 0.05;
                else if (targetError < 4 * nearTargetDistance)
                    fraction = 0.10;
                else if (targetError < 10 * nearTargetDistance)
                    fraction = 0.25;
                else
                    fraction = Clamp(remoteEffectFraction, MinimumRemoteEffectFraction, MaximumRemoteEffectFraction);

                // A predicted reversal is treated as a fine trim even when its
                // unscaled effect is large, preventing rapid branch-to-branch
                // oscillation.
                return reversesPreviousPulse ? Math.Min(fraction, 0.05) : fraction;
            }

            public static double UpdateRemoteEffectFraction(double currentFraction, double commandedFraction,
                double observedFraction, ref int verifiedResponses)
            {
                currentFraction = Clamp(currentFraction, MinimumRemoteEffectFraction, MaximumRemoteEffectFraction);
                if (!IsFinite(commandedFraction) || !IsFinite(observedFraction) || commandedFraction <= 0)
                {
                    verifiedResponses = 0;
                    return currentFraction;
                }

                // Raise gain only when the measured endpoint movement matches
                // the predicted surface effect closely on two successive pulses.
                if (observedFraction >= 0.75 * commandedFraction &&
                    observedFraction <= 1.25 * commandedFraction)
                {
                    verifiedResponses++;
                    if (verifiedResponses >= 2)
                    {
                        verifiedResponses = 0;
                        return Math.Min(MaximumRemoteEffectFraction, currentFraction + RemoteEffectFractionStep);
                    }

                    return currentFraction;
                }

                verifiedResponses = 0;
                // A weak or adverse measured response means the local model is
                // not trustworthy enough for a large next correction.
                if (observedFraction < 0.25 * commandedFraction)
                    return Math.Max(MinimumRemoteEffectFraction, 0.5 * currentFraction);

                return currentFraction;
            }

            private static double Clamp(double value, double minimum, double maximum) =>
                Math.Max(minimum, Math.Min(maximum, value));

            private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
