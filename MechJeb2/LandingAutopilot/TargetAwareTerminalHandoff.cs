using System;

namespace MuMech.Landing
{
    internal readonly struct TargetAwareTerminalHandoff
    {
        internal readonly double LocalTerrainASL;
        internal readonly double RemainingClearance;
        internal readonly double EndVerticalSpeed;
        internal readonly double EndSurfaceSpeed;
        internal readonly double EndHorizontalSpeed;
        internal readonly double ControllerTransitionClearance;
        internal readonly double ControllerTransitionVerticalSpeed;
        internal readonly double ControllerTransitionSurfaceSpeed;
        internal readonly double OptimisticVerticalStoppingDistance;
        internal readonly double IdealHoverHorizontalStoppingDistance;
        internal readonly double IdealHoverHorizontalStoppingTime;
        internal readonly bool NecessaryControlBoundPasses;

        private TargetAwareTerminalHandoff(double localTerrainASL, double remainingClearance,
            double endVerticalSpeed, double endSurfaceSpeed, double endHorizontalSpeed,
            double transitionClearance,
            double transitionVerticalSpeed, double transitionSurfaceSpeed,
            double optimisticStoppingDistance, double idealHoverHorizontalStoppingDistance,
            double idealHoverHorizontalStoppingTime,
            bool necessaryControlBoundPasses)
        {
            LocalTerrainASL = localTerrainASL;
            RemainingClearance = remainingClearance;
            EndVerticalSpeed = endVerticalSpeed;
            EndSurfaceSpeed = endSurfaceSpeed;
            EndHorizontalSpeed = endHorizontalSpeed;
            ControllerTransitionClearance = transitionClearance;
            ControllerTransitionVerticalSpeed = transitionVerticalSpeed;
            ControllerTransitionSurfaceSpeed = transitionSurfaceSpeed;
            OptimisticVerticalStoppingDistance = optimisticStoppingDistance;
            IdealHoverHorizontalStoppingDistance = idealHoverHorizontalStoppingDistance;
            IdealHoverHorizontalStoppingTime = idealHoverHorizontalStoppingTime;
            NecessaryControlBoundPasses = necessaryControlBoundPasses;
        }

        internal static TargetAwareTerminalHandoff Assess(AirlessTargetAwareSnapshot snapshot,
            AirlessTargetAwareOutput output, double localTerrainASL)
        {
            if (output == null || !output.ReachedHandoff || !Finite(localTerrainASL))
                return default(TargetAwareTerminalHandoff);
            AirlessTargetAwareState endpoint = output.End;
            double endRadius = endpoint.Position.magnitude;
            double clearance = endRadius - snapshot.BodyRadius - localTerrainASL;
            double endVertical = Vector3d.Dot(endpoint.Velocity, endpoint.Position.normalized);
            if (output.UsesV1ControlModel)
            {
                double gravity = snapshot.BodyMu / (endRadius * endRadius);
                double availableAcceleration = snapshot.MaximumThrust / snapshot.InitialMass - gravity;
                double stoppingDistance = availableAcceleration > 0 ?
                    Math.Pow(Math.Max(0, -endVertical), 2) / (2 * availableAcceleration) :
                    double.PositiveInfinity;
                double surfaceSpeed = (endpoint.Velocity -
                    Vector3d.Cross(snapshot.AngularVelocity, endpoint.Position)).magnitude;
                Vector3d surfaceVelocity = endpoint.Velocity -
                    Vector3d.Cross(snapshot.AngularVelocity, endpoint.Position);
                double horizontalSpeed = Vector3d.Exclude(endpoint.Position.normalized,
                    surfaceVelocity).magnitude;
                // KHV commands a 0.2 lateral-to-vertical tilt while hovering.
                // Its ideal lateral deceleration is 0.2 g, not full thrust.
                // This is a horizontal travel distance; comparing it with
                // vertical clearance would reject or accept for the wrong reason.
                double idealHoverHorizontalStoppingDistance = gravity > 0 ?
                    horizontalSpeed * horizontalSpeed / (0.4 * gravity) :
                    double.PositiveInfinity;
                double idealHoverHorizontalStoppingTime = gravity > 0 ?
                    horizontalSpeed / (0.2 * gravity) : double.PositiveInfinity;
                bool passes = clearance >= 0 &&
                    snapshot.MaximumThrust / snapshot.InitialMass >
                    gravity * Math.Sqrt(1 + 0.2 * 0.2) &&
                    stoppingDistance <= clearance &&
                    Finite(surfaceSpeed);
                return new TargetAwareTerminalHandoff(localTerrainASL, clearance, endVertical,
                    surfaceSpeed, horizontalSpeed, clearance, endVertical, surfaceSpeed,
                    stoppingDistance, idealHoverHorizontalStoppingDistance,
                    idealHoverHorizontalStoppingTime, passes);
            }
            double transitionRadius = snapshot.BodyRadius + localTerrainASL + 205;
            for (int i = 1; i < output.Trajectory.Count; ++i)
            {
                AirlessTargetAwareState earlier = output.Trajectory[i - 1];
                AirlessTargetAwareState later = output.Trajectory[i];
                if (earlier.Position.magnitude < transitionRadius ||
                    later.Position.magnitude >= transitionRadius)
                    continue;
                double fraction = (earlier.Position.magnitude - transitionRadius) /
                    (earlier.Position.magnitude - later.Position.magnitude);
                Vector3d position = earlier.Position + fraction * (later.Position - earlier.Position);
                Vector3d velocity = earlier.Velocity + fraction * (later.Velocity - earlier.Velocity);
                double vertical = Vector3d.Dot(velocity, position.normalized);
                double surfaceSpeed = (velocity - Vector3d.Cross(snapshot.AngularVelocity, position)).magnitude;
                double gravity = snapshot.BodyMu / (transitionRadius * transitionRadius);
                double availableAcceleration = snapshot.MaximumThrustAcceleration - gravity;
                double stoppingDistance = availableAcceleration > 0 ?
                    Math.Pow(Math.Max(0, -vertical), 2) / (2 * availableAcceleration) :
                    double.PositiveInfinity;
                bool passes = clearance >= 0 && snapshot.MaximumThrustAcceleration > gravity &&
                    stoppingDistance <= 205 && Finite(surfaceSpeed) && Finite(endVertical);
                return new TargetAwareTerminalHandoff(localTerrainASL, clearance, endVertical,
                    output.EndSurfaceSpeed,
                    Vector3d.Exclude(position.normalized,
                        velocity - Vector3d.Cross(snapshot.AngularVelocity, position)).magnitude,
                    205, vertical, surfaceSpeed, stoppingDistance,
                    double.NaN, double.NaN, passes);
            }
            return new TargetAwareTerminalHandoff(localTerrainASL, clearance, endVertical,
                output.EndSurfaceSpeed, double.NaN, double.NaN, double.NaN, double.NaN,
                double.PositiveInfinity, double.NaN, double.NaN, false);
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
