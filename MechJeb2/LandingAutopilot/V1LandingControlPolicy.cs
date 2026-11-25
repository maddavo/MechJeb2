using System;
using UnityEngine;

namespace MuMech.Landing
{
    // Pure decisions shared by the live V1 steps and the airless forecast.
    // The forecast supplies propagated state and keeps uncertain future
    // attitude or terrain explicit in its publication gates.
    internal static class V1LandingControlPolicy
    {
        internal static double PlaneChangeTargetTrueAnomaly(double currentRadians, double targetAngleDegrees) =>
            currentRadians + targetAngleDegrees * (Math.PI / 180);

        internal static bool ShouldTriggerGeometricDeorbit(double targetNormalAngle,
            double targetAheadAngle, double planeChangeAngle) =>
            targetNormalAngle < 10.0 ||
            (targetAheadAngle < 90.0 && targetAheadAngle > 60.0 && planeChangeAngle < 90.0);

        internal static double GeometricDeorbitThrottle(double burnMagnitude,
            double maximumThrustAcceleration, double currentThrustAcceleration,
            double engineResponseTime, double bodyGeeASL)
        {
            const double MinimumDeorbitAcceleration = 3.0;
            const double MaximumDeorbitTwr = 4.0;
            const double DeorbitBurnTimeConstant = 0.5;
            double bodyAwareMaximum = Math.Max(MinimumDeorbitAcceleration,
                MaximumDeorbitTwr * bodyGeeASL * 9.81);
            double cappedAcceleration = Math.Min(maximumThrustAcceleration, bodyAwareMaximum);
            double responseDeltaV = currentThrustAcceleration * engineResponseTime;
            double desiredAcceleration = Math.Max(0.0,
                (burnMagnitude - responseDeltaV) / DeorbitBurnTimeConstant);
            return Math.Min(desiredAcceleration / maximumThrustAcceleration,
                cappedAcceleration / maximumThrustAcceleration);
        }

        internal static bool GeometricDeorbitComplete(double burnMagnitude,
            double maximumThrustAcceleration, double bodyGeeASL,
            double fixedDeltaTime, double engineResponseTime)
        {
            const double MinimumDeorbitAcceleration = 3.0;
            const double MaximumDeorbitTwr = 4.0;
            const double MinimumTerminalDeltaV = 0.05;
            double bodyAwareMaximum = Math.Max(MinimumDeorbitAcceleration,
                MaximumDeorbitTwr * bodyGeeASL * 9.81);
            double cappedAcceleration = Math.Min(maximumThrustAcceleration, bodyAwareMaximum);
            double terminalDeltaV = Math.Max(MinimumTerminalDeltaV,
                cappedAcceleration * (fixedDeltaTime + engineResponseTime));
            return burnMagnitude <= terminalDeltaV;
        }

        internal static bool PostPulsePredictionSettled(long currentVersion,
            long versionAtPulseCompletion, double predictionInputUT,
            double pulseCompletionUT) =>
            currentVersion > versionAtPulseCompletion &&
            predictionInputUT >= pulseCompletionUT + 0.75;

        internal static bool CorrectionResetRequiresSettling(bool waitingForSettledInput,
            double thrustAcceleration, double appliedThrottle, double activePulseDeliveredDv) =>
            waitingForSettledInput || thrustAcceleration > 0 || appliedThrottle > 0 || activePulseDeliveredDv > 0;

        // V1WarpPreparation starts a passive turn observation at five degrees.
        // Holding an already aligned aim cannot establish an unknown turn rate.
        internal static bool CanObserveCorrectionPreparation(double aimAngle) =>
            FiniteCorrection(aimAngle) && aimAngle >= 5;

        internal static bool CorrectionPreparationUnavailable(bool observing, double aimAngle) =>
            !observing && !CanObserveCorrectionPreparation(aimAngle);

        internal static double CorrectionRefreshSeconds(double now, double inputUT,
            double previousFreshPublicationUT, double settledReadyUT, double postBurnPeakSeconds)
        {
            if (!FiniteCorrection(now) || !FiniteCorrection(inputUT)) return double.NaN;
            double age = Math.Max(0, now - inputUT);
            double anchor = previousFreshPublicationUT;
            if (FiniteCorrection(settledReadyUT))
                anchor = FiniteCorrection(anchor) ? Math.Max(anchor, settledReadyUT) : settledReadyUT;
            double cadence = FiniteCorrection(anchor) ? Math.Max(0, now - anchor) : age;
            return Math.Max(Math.Max(age, cadence), FiniteCorrection(postBurnPeakSeconds) ? postBurnPeakSeconds : 0);
        }

        internal static double CourseCorrectionHandoffDistance(double bodyRadius,
            double surfaceSpeed, double limitedMaxThrustAcceleration)
        {
            double downrangeCaptureDistance = Math.Max(100, bodyRadius * 0.005);
            double availableAcceleration = Math.Max(0.1, limitedMaxThrustAcceleration);
            double brakingDistance = surfaceSpeed * surfaceSpeed / (2 * availableAcceleration);
            return Math.Max(downrangeCaptureDistance,
                Math.Min(bodyRadius * 0.01, brakingDistance));
        }

        internal static double MaximumCourseCorrectionPulse(double targetError,
            double bodyRadius, bool hasLastPulseDirection, double directionChangeAngle)
        {
            double maximumPulseDv = 1.0;
            double nearTargetDistance = Math.Max(250, bodyRadius * 0.01);
            if (targetError < nearTargetDistance)
                maximumPulseDv = 0.1;
            else if (targetError < 4 * nearTargetDistance)
                maximumPulseDv = 0.25;
            if (hasLastPulseDirection && directionChangeAngle > 90)
                maximumPulseDv = Math.Min(maximumPulseDv, 0.1);
            return maximumPulseDv;
        }

        // Match the existing ThrustForDv law: full acceleration, exponential
        // taper (0.5 s plus engine response), then its 1% actuator floor. User
        // and physical engine floors shorten delivery; they are never bypassed.
        internal static double CorrectionPulseSeconds(double dv, double acceleration,
            double maximumAcceleration, double engineResponse, double minimumThrottle,
            double minimumAcceleration, double tick)
        {
            if (!FiniteCorrection(dv) || dv <= 0) return 0;
            if (!FiniteCorrection(acceleration) || acceleration <= 0 ||
                !FiniteCorrection(maximumAcceleration) || maximumAcceleration <= 0)
                return double.PositiveInfinity;
            double tau = 0.5 + Math.Max(0, engineResponse);
            double floor = Math.Min(acceleration, Math.Max(minimumAcceleration,
                maximumAcceleration * Math.Max(0.01, minimumThrottle)));
            double fullBoundary = acceleration * tau;
            double floorBoundary = floor * tau;
            double remaining = dv;
            double seconds = Math.Max(0, engineResponse) * 2 + Math.Max(0, tick);
            if (remaining > fullBoundary)
            {
                seconds += (remaining - fullBoundary) / acceleration;
                remaining = fullBoundary;
            }
            if (remaining > Math.Max(0.05, floorBoundary))
            {
                double end = Math.Max(0.05, floorBoundary);
                seconds += tau * Math.Log(remaining / end);
                remaining = end;
            }
            if (remaining > 0.05) seconds += (remaining - 0.05) / floor;
            return seconds;
        }

        internal static double CorrectionBrakeDeadline(bool firstThrustValidated,
            double firstThrustUT, double controllerReferenceUT) =>
            firstThrustValidated && FiniteCorrection(firstThrustUT) ? firstThrustUT :
            FiniteCorrection(controllerReferenceUT) ? controllerReferenceUT - 5 : double.NaN;

        internal static double CorrectionCycleReserve(double correctionAimSeconds,
            double predictionRefreshSeconds, double brakePreparationSeconds,
            double warpOverrunSeconds, double tick) =>
            correctionAimSeconds >= 0 && predictionRefreshSeconds >= 0 && brakePreparationSeconds >= 0 &&
            FiniteCorrection(correctionAimSeconds) && FiniteCorrection(predictionRefreshSeconds) &&
            FiniteCorrection(brakePreparationSeconds) ?
                correctionAimSeconds + 0.75 + 2 * predictionRefreshSeconds +
                brakePreparationSeconds + Math.Max(0, warpOverrunSeconds) + Math.Max(0, tick) :
                double.PositiveInfinity;

        internal static double SelectCorrectionPulse(double requiredDv, double endpointError,
            double localMetresPerDv, double trustedDv, double directionChangeDegrees,
            double availableSeconds, double reserveSeconds, Func<double, double> duration)
        {
            const double PulseCompletionDv = 0.05;
            const double MinimumUsefulDeliveredDv = 0.05;
            const double MinimumCommandDv = PulseCompletionDv + MinimumUsefulDeliveredDv;
            if (!FiniteCorrection(requiredDv) || !FiniteCorrection(endpointError) ||
                !FiniteCorrection(localMetresPerDv) || !FiniteCorrection(trustedDv) ||
                requiredDv <= 0.05 || endpointError <= 0 || localMetresPerDv <= 0 ||
                trustedDv <= 0 || !FiniteCorrection(availableSeconds) ||
                !FiniteCorrection(reserveSeconds) || availableSeconds <= reserveSeconds || duration == null)
                return 0;
            // A half-residual step leaves room to measure the nonlinear forecast
            // response. The trust bound is validated against perturbed geometry,
            // and later independently observed powered endpoints, by the caller.
            double pulse = Math.Min(requiredDv / 2, Math.Min(trustedDv,
                endpointError / (2 * localMetresPerDv)));
            if (directionChangeDegrees > 90) pulse = Math.Min(pulse, trustedDv / 2);
            if (pulse < MinimumCommandDv) return 0;
            if (duration(pulse) + reserveSeconds <= availableSeconds) return pulse;
            if (duration(MinimumCommandDv) + reserveSeconds > availableSeconds) return 0;
            double lo = MinimumCommandDv, hi = pulse;
            for (int i = 0; i < 24; ++i)
            {
                double mid = (lo + hi) / 2;
                if (duration(mid) + reserveSeconds <= availableSeconds) lo = mid;
                else hi = mid;
            }
            return lo;
        }

        // Selection and geometry use delivered impulse. ThrustForDv consumes
        // command residual; its existing cutoff leaves 0.05 m/s undelivered.
        internal static double CorrectionCommandForDelivery(double deliveredDv) => deliveredDv + 0.05;

        internal static double CorrectionDeliveryResolution(double maximumAcceleration,
            double minimumAcceleration, double minimumThrottle, double tick, double observedTailDv) =>
            Math.Max(1e-9, Math.Max(minimumAcceleration,
                maximumAcceleration * Math.Max(0.01, minimumThrottle)) * tick + Math.Max(0, observedTailDv));

        internal static double SelectCorrectionDelivery(double requiredDv, double endpointError,
            double completionError, double localMetresPerDv, double trustedDv, double directionChangeDegrees,
            double availableSeconds, double reserveSeconds, double resolution, Func<double,double> duration,
            out string reason)
        {
            reason = "invalid_correction_inputs";
            if (!FiniteCorrection(requiredDv) || !FiniteCorrection(endpointError) ||
                !FiniteCorrection(localMetresPerDv) || requiredDv <= 0 || localMetresPerDv <= 0 ||
                !FiniteCorrection(resolution) || resolution <= 0 || duration == null) return 0;
            if (endpointError < completionError) { reason = "target_converged"; return 0; }
            if (!FiniteCorrection(trustedDv) || trustedDv <= 0) { reason = "correction_geometry_rejected"; return 0; }
            if (!FiniteCorrection(reserveSeconds)) { reason = "correction_preparation_unavailable"; return 0; }
            if (!FiniteCorrection(availableSeconds) || availableSeconds <= reserveSeconds)
            { reason = "preparation_deadline"; return 0; }
            double limit = Math.Min(requiredDv, Math.Min(trustedDv, endpointError / localMetresPerDv));
            if (directionChangeDegrees > 90) limit = Math.Min(limit, trustedDv / 2);
            double delivered = Math.Min(limit, Math.Min(requiredDv / 2, endpointError / (2 * localMetresPerDv)));
            if (delivered < resolution)
            {
                // A finishing step may exceed half demand, but never the
                // independently screened geometry or the full residual.
                delivered = Math.Min(limit, Math.Max(resolution,
                    (endpointError - completionError) / localMetresPerDv + resolution));
            }
            if (delivered < resolution) { reason = "insufficient_actuator_resolution"; return 0; }
            if (duration(delivered) + reserveSeconds > availableSeconds)
            {
                if (duration(resolution) + reserveSeconds > availableSeconds)
                { reason = "preparation_deadline"; return 0; }
                double lo = resolution, hi = delivered;
                for (int i=0;i<24;i++)
                {
                    double mid=(lo+hi)/2;
                    if (duration(mid)+reserveSeconds<=availableSeconds) lo=mid; else hi=mid;
                }
                delivered=lo;
            }
            reason = "none";
            return delivered;
        }

        internal static double CorrectionResponseTrust(double priorTrust, double deliveredDv,
            double responseDisagreement, double allowedDisagreement, bool sameModel,
            double directionChangeDegrees)
        {
            // A changed brake/terrain model cannot calibrate this forecast's
            // response. Reacquire trust using fresh impact-preserving geometry.
            if (!sameModel) return double.NaN;
            if (!FiniteCorrection(responseDisagreement) ||
                responseDisagreement > allowedDisagreement || directionChangeDegrees > 90)
                return Math.Min(1, priorTrust / 2);
            return Math.Max(priorTrust, 2 * deliveredDv);
        }

        internal static bool SingleCorrectionForecastSufficient(bool sameModel, double disagreement,
            double allowedDisagreement, double directionChange, bool capabilityUnchanged) =>
            sameModel && capabilityUnchanged && FiniteCorrection(disagreement) &&
            disagreement <= allowedDisagreement && directionChange <= 20;

        private static bool FiniteCorrection(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        internal static bool ShouldStartDeceleration(double surfaceSpeed, double maximumAllowedSpeed) =>
            surfaceSpeed > 0.9 * maximumAllowedSpeed;

        internal static double SafeDescentMaximumSpeed(double radius, double terrainRadius,
            double gravity, double thrustAcceleration) =>
            0.9 * Math.Sqrt(2 * (thrustAcceleration - gravity) *
                            (radius - terrainRadius));

        internal static bool BelowTerminalHandoff(double altitudeASL, double decelerationEndASL) =>
            altitudeASL < decelerationEndASL + 5;

        internal static bool ShouldCoastToBrake(double brakeUT, double currentUT, bool burnTriggered) =>
            brakeUT - currentUT > 5 && !burnTriggered;

        // A powered pulse has a tighter pointing gate than warp, with a
        // bounded rotation rate (less than 0.6 degrees/s).
        internal static bool CorrectionAimReady(double error, double angularSpeed) =>
            error >= 0 && error < 2 && angularSpeed >= 0 && angularSpeed < 0.01;

        internal static double HigherImpactHandoff(double currentASL, double contactTerrainASL,
            double supportOffset) =>
            FiniteCorrection(contactTerrainASL) && FiniteCorrection(supportOffset) &&
            supportOffset >= 0 && contactTerrainASL + supportOffset >= currentASL + 5 ?
                Math.Max(currentASL, contactTerrainASL + 200) : currentASL;

        internal static bool BrakeWarpReady(double attitudeErrorDegrees, double angularSpeed) =>
            attitudeErrorDegrees < 5 && angularSpeed < 0.001;

        internal static double DeorbitPreparationRangeError(Vector3d position, Vector3d orbitalVelocity,
            Vector3d targetBCI, Vector3d angularVelocity, double coastSeconds,
            double bodyRadius, double decelerationEndASL, double acceleration)
        {
            Vector3d target = angularVelocity.sqrMagnitude > 0 ?
                QuaternionD.AngleAxis(coastSeconds * angularVelocity.magnitude * UtilMath.Rad2Deg,
                    angularVelocity.normalized) * targetBCI : targetBCI;
            Vector3d velocity = orbitalVelocity - Vector3d.Cross(angularVelocity, position);
            Vector3d toTarget = target - position;
            // Projected range also vanishes at the antipode. It is not the
            // upcoming target approach and must not be an orientation aim.
            if (acceleration <= 0 || Vector3d.Dot(position, target) <= 0 ||
                Vector3d.Dot(toTarget, velocity) <= 0)
                return double.PositiveInfinity;
            Vector3d up = position.normalized;
            double horizontalSpeed = Vector3d.Exclude(up, velocity).magnitude;
            double trigger = Math.Max(horizontalSpeed * horizontalSpeed / acceleration,
                position.magnitude - bodyRadius - decelerationEndASL);
            return Vector3d.Exclude(up, toTarget).magnitude - trigger;
        }

        internal static double UpcomingDeorbitTrigger(Func<double, double> approachingRangeError,
            double currentUT, double horizon)
        {
            // Bounded coast-orbit geometry only. The callback excludes the
            // receding half of the orbit, where projected range is ambiguous.
            if (double.IsNaN(horizon) || double.IsInfinity(horizon) || horizon <= 0)
                return double.NaN;
            int evaluations = 0;
            double Evaluate(double ut) { evaluations++; return approachingRangeError(ut); }
            double left = currentUT;
            double prior = Evaluate(left);
            if (prior <= 0) return currentUT;
            // Leave room for boundary refinement and root bisection: never
            // exceed 128 geometry calls even for discontinuous inputs.
            for (int i = 1; i <= 64 && evaluations < 96; ++i)
            {
                double right = currentUT + horizon * i / 64;
                double error = Evaluate(right);
                if (!double.IsInfinity(prior) && prior > 0 && double.IsPositiveInfinity(error))
                {
                    // A narrow trigger window can fall wholly between coarse
                    // samples just before the target becomes receding. Locate
                    // that boundary rather than skipping to the next orbit.
                    double finiteSide = left, excludedSide = right;
                    for (int j = 0; j < 16; ++j)
                    {
                        double middle = (finiteSide + excludedSide) / 2;
                        double value = Evaluate(middle);
                        if (double.IsPositiveInfinity(value)) excludedSide = middle;
                        else if (value <= 0) { right = middle; error = value; break; }
                        else finiteSide = middle;
                    }
                }
                if (error <= 0 && prior > 0)
                {
                    for (int j = 0; j < 16; ++j)
                    {
                        double middle = (left + right) / 2;
                        if (Evaluate(middle) <= 0) right = middle;
                        else left = middle;
                    }
                    return right;
                }
                left = right;
                prior = error;
            }
            return double.NaN;
        }

        internal static double PreparedDeorbitWarpUT(double triggerUT, double now,
            double measuredPreparationSeconds, bool newCommandReady, bool farFromTrigger)
        {
            double exit = triggerUT - measuredPreparationSeconds;
            return newCommandReady && farFromTrigger && measuredPreparationSeconds >= 0 &&
                !double.IsInfinity(exit) && exit > now ? exit : double.NaN;
        }

        internal static Vector3d BrakeThrustDirection(Vector3d surfaceVelocity,
            Vector3d courseCorrection, double limitedMaxThrustAcceleration)
        {
            Vector3d desiredThrustVector = -surfaceVelocity.normalized;
            double correctionAngle = courseCorrection.magnitude / (2.0 * limitedMaxThrustAcceleration);
            correctionAngle = Math.Min(0.1, correctionAngle);
            return (desiredThrustVector + correctionAngle * courseCorrection.normalized).normalized;
        }

        internal static bool BrakeAttitudeGate(double upwardVelocity, double forwardAlignment) =>
            upwardVelocity > 0 || forwardAlignment < 0.75;

        internal static float BrakingThrottle(double controlledSpeed, double desiredSpeed,
            double desiredSpeedAfterDt, double deltaT, double minimumAcceleration,
            double maximumAcceleration)
        {
            const double SPEED_CORRECTION_TIME_CONSTANT = 0.3;
            double speedError = desiredSpeed - controlledSpeed;
            double desiredAccel = speedError / SPEED_CORRECTION_TIME_CONSTANT +
                (desiredSpeedAfterDt - desiredSpeed) / deltaT;
            if (maximumAcceleration - minimumAcceleration > 0)
                return Mathf.Clamp((float)((desiredAccel - minimumAcceleration) /
                    (maximumAcceleration - minimumAcceleration)), 0.0f, 1.0f);
            return 0;
        }

        internal static float HoverThrottle(double controlledVerticalSpeed,
            double localGravity, double forwardUpAlignment, double maximumThrustAcceleration)
        {
            const double DESIRED_SPEED = 0;
            double speedError = DESIRED_SPEED - controlledVerticalSpeed;
            const double SPEED_CORRECTION_TIME_CONSTANT = 1.0;
            double desiredAccel = speedError / SPEED_CORRECTION_TIME_CONSTANT;
            double minAccel = -localGravity;
            double maxAccel = -localGravity + forwardUpAlignment * maximumThrustAcceleration;
            if (maxAccel - minAccel > 0)
                return Mathf.Clamp((float)((desiredAccel - minAccel) /
                    (maxAccel - minAccel)), 0.0f, 1.0f);
            return 0;
        }

        internal static Vector3d HorizontalKillThrustDirection(Vector3d surfaceVelocity,
            Vector3d up)
        {
            Vector3d horizontal = Vector3d.Exclude(up, surfaceVelocity);
            return horizontal.sqrMagnitude > 1e-12 ?
                (up - 0.2 * horizontal.normalized).normalized : up;
        }

        internal static double UprightColliderOffset(Vector3d centerFromCoM,
            Vector3d extents, Vector3d vesselForward)
        {
            Vector3d axis = vesselForward.normalized;
            return Math.Max(0, -Vector3d.Dot(centerFromCoM, axis) +
                Math.Abs(axis.x) * extents.x + Math.Abs(axis.y) * extents.y + Math.Abs(axis.z) * extents.z);
        }

        internal static double FinalDescentSpeed(double altitudeAboveTerrain,
            double limitedMaxThrustAcceleration, double localGravity,
            double touchdownSpeed, bool targetedAirless = false)
        {
            const double MinimumTerminalDescentSpeed = 5.0;
            const double MaximumTerminalDescentSpeed = 25.0;
            const double TerminalDescentGravityTime = 4.0;
            double netBrakingAcceleration = Math.Max(0,
                limitedMaxThrustAcceleration - localGravity);
            // KEEP_VERTICAL uses a cumulative PID throttle, so its commanded
            // thrust does not jump to the available maximum at the point of
            // the ideal stopping-distance curve. The captured Mun handoff is
            // replayed with that same PID before this lead factor is accepted.
            double brakingDistanceSpeed = Math.Sqrt(2 * netBrakingAcceleration *
                Math.Max(0, altitudeAboveTerrain)) *
                (targetedAirless ? 0.40 : 0.90);
            double bodyAwareSpeedCap = Math.Max(MinimumTerminalDescentSpeed,
                Math.Min(MaximumTerminalDescentSpeed, TerminalDescentGravityTime * localGravity));
            double desiredSpeed = -Math.Min(brakingDistanceSpeed, bodyAwareSpeedCap);
            if (targetedAirless)
            {
                // The captured heavy Mun vessel contacted its legs about 2.4 m
                // before AltitudeBottom reached zero. At 6.24 m/s that is 0.39 s.
                // A half-second clearance reserve covers that measured contact,
                // while 3.2 s is the four-time-constant settling envelope of the
                // existing cumulative PID (Kd=.05, physics step=.02). Recorded
                // Mun/Minmus replays exercise this bound without changing gains.
                const double ContactReserveSeconds = 0.5;
                const double TerminalSettlingSeconds = 3.2;
                double approach = Math.Max(touchdownSpeed,
                    (altitudeAboveTerrain - bodyAwareSpeedCap * ContactReserveSeconds) /
                    TerminalSettlingSeconds);
                desiredSpeed = Math.Max(desiredSpeed, -approach);
            }
            return Math.Min(-touchdownSpeed, desiredSpeed);
        }
    }

    // Preparation is one physical turn, not a succession of osculating
    // element snapshots. Readiness is always checked against the live command.
    internal sealed class V1DeorbitPreparation
    {
        internal double StartedUT { get; private set; } = double.NaN;
        internal double Seconds { get; private set; }
        internal bool Complete { get; private set; }
        internal bool LiveRetrograde { get; private set; }

        internal void Start(double now)
        {
            if (double.IsNaN(StartedUT)) StartedUT = now;
        }

        internal void Observe(bool ready, bool physicsRunning, double now, double deltaT)
        {
            if (!physicsRunning || LiveRetrograde) return;
            if (!ready && Complete)
            {
                StartedUT = now;
                Complete = false;
            }
            else if (ready && !Complete)
            {
                Seconds = Math.Max(Seconds, Math.Max(deltaT, now - StartedUT));
                Complete = true;
            }
        }

        internal bool UseLiveRetrograde(double triggerUT, double now, double reserve = double.NaN)
        {
            if (Complete && !double.IsNaN(triggerUT) && !double.IsInfinity(triggerUT) &&
                now >= triggerUT - (double.IsNaN(reserve) ? Seconds : reserve))
                LiveRetrograde = true;
            // Once the reserve begins, never aim at the following orbit.
            return LiveRetrograde;
        }
    }
}
