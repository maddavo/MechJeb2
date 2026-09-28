using System;
using UnityEngine;

namespace MuMech.Landing
{
    // Pure decisions shared by the live V1 steps and the airless forecast.
    // The forecast supplies propagated state and keeps uncertain future
    // attitude or terrain explicit in its publication gates.
    internal static class V1LandingControlPolicy
    {
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

        internal static bool BrakeWarpReady(double attitudeErrorDegrees, double angularSpeed) =>
            attitudeErrorDegrees < 5 && angularSpeed < 0.001;

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

        internal static double FinalDescentSpeed(double altitudeAboveTerrain,
            double limitedMaxThrustAcceleration, double localGravity, double touchdownSpeed)
        {
            const double MinimumTerminalDescentSpeed = 5.0;
            const double MaximumTerminalDescentSpeed = 25.0;
            const double TerminalDescentGravityTime = 4.0;
            double netBrakingAcceleration = Math.Max(0,
                limitedMaxThrustAcceleration - localGravity);
            double brakingDistanceSpeed = Math.Sqrt(2 * netBrakingAcceleration *
                Math.Max(0, altitudeAboveTerrain)) * 0.90;
            double bodyAwareSpeedCap = Math.Max(MinimumTerminalDescentSpeed,
                Math.Min(MaximumTerminalDescentSpeed, TerminalDescentGravityTime * localGravity));
            double desiredSpeed = -Math.Min(brakingDistanceSpeed, bodyAwareSpeedCap);
            return Math.Min(-touchdownSpeed, desiredSpeed);
        }
    }
}
