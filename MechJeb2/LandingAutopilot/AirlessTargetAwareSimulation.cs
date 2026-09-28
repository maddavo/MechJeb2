using System;
using System.Collections.Generic;

namespace MuMech.Landing
{
    // All fields are copied on the flight thread. The worker uses no Vessel,
    // CelestialBody, Unity, terrain, or mutable Orbit objects.
    internal readonly struct AirlessTargetAwareSnapshot
    {
        internal readonly double InputUT, EpochUT, BodyRadius, BodyMu, BodyGeeASL, RotationPeriod;
        internal readonly double TargetLatitude, TargetLongitude, TargetTerrainASL;
        internal readonly double DecelEndASL, MaximumThrustAcceleration, Dt, MinDt, MaxOrbits;
        internal readonly double MinimumTerrainASL, MaximumTerrainASL;
        internal readonly Vector3d Position, Velocity, AngularVelocity;
        internal readonly Vector3d Axis0, Axis90, AxisNorth;

        internal AirlessTargetAwareSnapshot(double inputUT, double epochUT, double bodyRadius,
            double bodyMu, double bodyGeeASL, double rotationPeriod, double targetLatitude,
            double targetLongitude, double targetTerrainASL, double decelEndASL,
            double maximumThrustAcceleration, double dt, double minDt, double maxOrbits,
            double minimumTerrainASL, double maximumTerrainASL, Vector3d position,
            Vector3d velocity, Vector3d angularVelocity, Vector3d axis0, Vector3d axis90,
            Vector3d axisNorth)
        {
            InputUT = inputUT;
            EpochUT = epochUT;
            BodyRadius = bodyRadius;
            BodyMu = bodyMu;
            BodyGeeASL = bodyGeeASL;
            RotationPeriod = rotationPeriod;
            TargetLatitude = targetLatitude;
            TargetLongitude = targetLongitude;
            TargetTerrainASL = targetTerrainASL;
            DecelEndASL = decelEndASL;
            MaximumThrustAcceleration = maximumThrustAcceleration;
            Dt = dt;
            MinDt = minDt;
            MaxOrbits = maxOrbits;
            MinimumTerrainASL = minimumTerrainASL;
            MaximumTerrainASL = maximumTerrainASL;
            Position = position;
            Velocity = velocity;
            AngularVelocity = angularVelocity;
            Axis0 = axis0;
            Axis90 = axis90;
            AxisNorth = axisNorth;
        }
    }

    internal readonly struct AirlessTargetAwareState
    {
        internal readonly Vector3d Position;
        internal readonly Vector3d Velocity;
        internal readonly double UT;

        internal AirlessTargetAwareState(Vector3d position, Vector3d velocity, double ut)
        {
            Position = position;
            Velocity = velocity;
            UT = ut;
        }
    }

    internal sealed class AirlessTargetAwareOutput
    {
        internal readonly double BrakeUT;
        internal readonly List<AirlessTargetAwareState> CoastSamples;
        internal readonly List<AirlessTargetAwareState> Trajectory;
        internal readonly bool ReachedHandoff;
        internal readonly double EndSurfaceSpeed;
        internal readonly double VirtualDeltaV;
        internal readonly int Steps;

        internal AirlessTargetAwareOutput(double brakeUT, List<AirlessTargetAwareState> coastSamples,
            List<AirlessTargetAwareState> trajectory, bool reachedHandoff,
            double endSurfaceSpeed, double virtualDeltaV, int steps)
        {
            BrakeUT = brakeUT;
            CoastSamples = coastSamples;
            Trajectory = trajectory;
            ReachedHandoff = reachedHandoff;
            EndSurfaceSpeed = endSurfaceSpeed;
            VirtualDeltaV = virtualDeltaV;
            Steps = steps;
        }

        internal AirlessTargetAwareState End => Trajectory[Trajectory.Count - 1];
    }

    internal static class AirlessTargetAwareSimulation
    {
        private const int MaximumCoastSteps = 20000;

        // Same airless gravity and speed-limiter integration as the existing
        // ReentrySimulation. Candidate 177's captured endpoint is the parity gate.
        internal static AirlessTargetAwareOutput Run(AirlessTargetAwareSnapshot snapshot, double brakeUT,
            bool normalizeHandoff = false)
        {
            ValidateSnapshot(snapshot);
            if (!Finite(brakeUT) || brakeUT < snapshot.InputUT)
                throw new ArgumentException("Invalid airless brake UT");

            var coastSamples = new List<AirlessTargetAwareState>();
            AirlessTargetAwareState state = new AirlessTargetAwareState(snapshot.Position,
                snapshot.Velocity, snapshot.InputUT);
            int coastSteps = 0;
            while (state.UT < brakeUT)
            {
                if (++coastSteps > MaximumCoastSteps)
                    throw new InvalidOperationException("Airless coast step limit exceeded");
                double step = Math.Min(1, brakeUT - state.UT);
                if (step < 1e-8) break;
                state = RK4(state, step, snapshot.BodyMu);
                if (state.Position.magnitude - snapshot.BodyRadius <= snapshot.MaximumTerrainASL)
                    coastSamples.Add(state);
            }

            var trajectory = new List<AirlessTargetAwareState> { state };
            Vector3d startPosition = state.Position;
            double dt = snapshot.Dt;
            double deltaV = 0;
            int steps = 0;
            double stopRadius = snapshot.BodyRadius + snapshot.TargetTerrainASL;
            double maximumTime = snapshot.MaxOrbits * 2 * Math.PI *
                Math.Sqrt(Math.Pow(startPosition.magnitude, 3) / snapshot.BodyMu);
            if (state.Position.magnitude < stopRadius)
                return new AirlessTargetAwareOutput(brakeUT, coastSamples, trajectory,
                    false, double.NaN, 0, 0);
            bool reached = false;
            while (!reached &&
                   state.UT - brakeUT <= maximumTime && steps <= 50000)
            {
                AirlessTargetAwareState previous = state;
                double previousDeltaV = deltaV;
                state = BS34(state, ref dt, snapshot.MinDt, startPosition,
                    stopRadius, snapshot.BodyMu, ref steps);
                Vector3d surfaceVelocity = state.Velocity -
                    Vector3d.Cross(snapshot.AngularVelocity, state.Position);
                double speed = surfaceVelocity.magnitude;
                double altitude = state.Position.magnitude - snapshot.BodyRadius - snapshot.DecelEndASL;
                double limitSquare = 2 * (snapshot.MaximumThrustAcceleration -
                    snapshot.BodyGeeASL * 9.81) * altitude;
                double allowed = limitSquare >= 0 ? 0.9 * Math.Sqrt(limitSquare) : double.NaN;
                if (speed > allowed)
                {
                    double change = Math.Min(speed - allowed, dt * snapshot.MaximumThrustAcceleration);
                    surfaceVelocity -= change * surfaceVelocity.normalized;
                    deltaV += change;
                    state = new AirlessTargetAwareState(state.Position,
                        surfaceVelocity + Vector3d.Cross(snapshot.AngularVelocity, state.Position), state.UT);
                }
                reached = state.Position.magnitude < stopRadius;
                if (reached && normalizeHandoff)
                {
                    double fraction = (previous.Position.magnitude - stopRadius) /
                        (previous.Position.magnitude - state.Position.magnitude);
                    fraction = Math.Max(0, Math.Min(1, fraction));
                    Vector3d position = previous.Position + fraction *
                        (state.Position - previous.Position);
                    position *= stopRadius / position.magnitude;
                    state = new AirlessTargetAwareState(position,
                        previous.Velocity + fraction * (state.Velocity - previous.Velocity),
                        previous.UT + fraction * (state.UT - previous.UT));
                    deltaV = previousDeltaV + fraction * (deltaV - previousDeltaV);
                }
                trajectory.Add(state);
            }
            double endSpeed = (state.Velocity - Vector3d.Cross(snapshot.AngularVelocity,
                state.Position)).magnitude;
            return new AirlessTargetAwareOutput(brakeUT, coastSamples, trajectory, reached,
                endSpeed, deltaV, steps);
        }

        internal static List<AirlessTargetAwareState> BallisticTerrainPass(AirlessTargetAwareSnapshot snapshot)
        {
            ValidateSnapshot(snapshot);
            var samples = new List<AirlessTargetAwareState>();
            AirlessTargetAwareState state = new AirlessTargetAwareState(snapshot.Position,
                snapshot.Velocity, snapshot.InputUT);
            double period = 2 * Math.PI * Math.Sqrt(Math.Pow(snapshot.Position.magnitude, 3) / snapshot.BodyMu);
            double horizon = snapshot.InputUT + period;
            double minimumRadius = snapshot.BodyRadius + snapshot.MinimumTerrainASL;
            bool seenDescending = false;
            int coastSteps = 0;
            while (state.UT < horizon && samples.Count < 1024)
            {
                if (++coastSteps > MaximumCoastSteps)
                    throw new InvalidOperationException("Ballistic coast step limit exceeded");
                double radialMotion = Vector3d.Dot(state.Position, state.Velocity);
                if (radialMotion < 0) seenDescending = true;
                if (state.Position.magnitude - snapshot.BodyRadius <= snapshot.MaximumTerrainASL)
                    samples.Add(state);
                if (state.Position.magnitude <= minimumRadius ||
                    (seenDescending && radialMotion >= 0))
                    break;
                state = RK4(state, Math.Min(1, horizon - state.UT), snapshot.BodyMu);
            }
            return samples;
        }

        private static void ValidateSnapshot(AirlessTargetAwareSnapshot snapshot)
        {
            if (!Finite(snapshot.InputUT) || !Finite(snapshot.EpochUT) ||
                !Finite(snapshot.BodyRadius) || snapshot.BodyRadius <= 0 ||
                !Finite(snapshot.BodyMu) || snapshot.BodyMu <= 0 ||
                !Finite(snapshot.BodyGeeASL) || snapshot.BodyGeeASL < 0 ||
                !Finite(snapshot.RotationPeriod) || snapshot.RotationPeriod == 0 ||
                !Finite(snapshot.TargetLatitude) || !Finite(snapshot.TargetLongitude) ||
                !Finite(snapshot.TargetTerrainASL) || !Finite(snapshot.DecelEndASL) ||
                !Finite(snapshot.MaximumThrustAcceleration) || snapshot.MaximumThrustAcceleration <= 0 ||
                !Finite(snapshot.Dt) || !Finite(snapshot.MinDt) || snapshot.MinDt <= 0 ||
                snapshot.Dt < snapshot.MinDt || !Finite(snapshot.MaxOrbits) || snapshot.MaxOrbits <= 0 ||
                !Finite(snapshot.MinimumTerrainASL) || !Finite(snapshot.MaximumTerrainASL) ||
                snapshot.MinimumTerrainASL > snapshot.MaximumTerrainASL ||
                !Finite(snapshot.Position) || !Finite(snapshot.Velocity) ||
                !Finite(snapshot.AngularVelocity) || !Finite(snapshot.Axis0) ||
                !Finite(snapshot.Axis90) || !Finite(snapshot.AxisNorth))
                throw new ArgumentException("Invalid airless prediction snapshot");
        }

        internal static AbsoluteVector ToAbsolute(Vector3d vector, double ut,
            AirlessTargetAwareSnapshot snapshot)
        {
            Vector3d unit = vector.normalized;
            double latitude = Math.Asin(Math.Max(-1, Math.Min(1,
                Vector3d.Dot(unit, snapshot.AxisNorth)))) * 180 / Math.PI;
            double longitude = Math.Atan2(Vector3d.Dot(unit, snapshot.Axis90),
                Vector3d.Dot(unit, snapshot.Axis0)) * 180 / Math.PI;
            longitude -= 360 * (ut - snapshot.EpochUT) / snapshot.RotationPeriod;
            return new AbsoluteVector { Latitude = latitude, Longitude = MuUtils.ClampDegrees180(longitude),
                Radius = vector.magnitude, UT = ut };
        }

        internal static Vector3d SurfaceDirection(double latitude, double longitude,
            AirlessTargetAwareSnapshot snapshot)
        {
            double lat = latitude * Math.PI / 180;
            double lon = longitude * Math.PI / 180;
            return Math.Cos(lat) * Math.Cos(lon) * snapshot.Axis0 +
                   Math.Cos(lat) * Math.Sin(lon) * snapshot.Axis90 +
                   Math.Sin(lat) * snapshot.AxisNorth;
        }

        private static AirlessTargetAwareState RK4(AirlessTargetAwareState state, double dt, double mu)
        {
            Vector3d a1 = Gravity(state.Position, mu);
            Vector3d v2 = state.Velocity + dt * 0.5 * a1;
            Vector3d a2 = Gravity(state.Position + dt * 0.5 * state.Velocity, mu);
            Vector3d v3 = state.Velocity + dt * 0.5 * a2;
            Vector3d a3 = Gravity(state.Position + dt * 0.5 * v2, mu);
            Vector3d v4 = state.Velocity + dt * a3;
            Vector3d a4 = Gravity(state.Position + dt * v3, mu);
            return new AirlessTargetAwareState(state.Position + dt / 6 *
                (state.Velocity + 2 * v2 + 2 * v3 + v4),
                state.Velocity + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4), state.UT + dt);
        }

        private static AirlessTargetAwareState BS34(AirlessTargetAwareState state, ref double dt,
            double minDt, Vector3d startPosition, double stopRadius, double mu, ref int steps)
        {
            for (int attempt = 0; attempt < 100; ++attempt)
            {
                steps++;
                Vector3d dv1 = dt * Gravity(state.Position, mu);
                Vector3d dx1 = dt * state.Velocity;
                Vector3d dv2 = dt * Gravity(state.Position + 0.5 * dx1, mu);
                Vector3d dx2 = dt * (state.Velocity + 0.5 * dv1);
                Vector3d dv3 = dt * Gravity(state.Position + 0.75 * dx2, mu);
                Vector3d dx3 = dt * (state.Velocity + 0.75 * dv2);
                Vector3d dv4 = dt * Gravity(state.Position + 2d / 9d * dx1 +
                    3d / 9d * dx2 + 4d / 9d * dx3, mu);
                Vector3d dx = (2 * dx1 + 3 * dx2 + 4 * dx3) / 9;
                Vector3d dv = (2 * dv1 + 3 * dv2 + 4 * dv3) / 9;
                Vector3d zv = (7 * dv1 + 6 * dv2 + 8 * dv3 + 3 * dv4) / 24;
                double error = Math.Max((zv - dv).magnitude, 1e-5);
                bool willCrossRadius = (state.Position + dx).magnitude < stopRadius;
                double nextDt = dt * (willCrossRadius ? 0.5 :
                    0.9 * Math.Pow(0.01 / error, 1d / 3d));
                nextDt = Math.Max(minDt, Math.Min(10, nextDt));
                double distanceSquared = (state.Position - startPosition).sqrMagnitude;
                if (distanceSquared < 1000 * 1000) nextDt = Math.Min(nextDt, 0.02);
                else if (distanceSquared < 5000 * 5000) nextDt = Math.Min(nextDt, 0.5);
                else if (distanceSquared < 10000 * 10000) nextDt = Math.Min(nextDt, 1);
                if ((error > 0.01 || willCrossRadius) && dt > minDt)
                {
                    dt = nextDt;
                    continue;
                }
                AirlessTargetAwareState accepted = new AirlessTargetAwareState(state.Position + dx,
                    state.Velocity + dv, state.UT + dt);
                dt = nextDt;
                return accepted;
            }
            throw new InvalidOperationException("Airless BS34 step did not converge");
        }

        private static Vector3d Gravity(Vector3d position, double mu) =>
            -(mu / position.sqrMagnitude) * position.normalized;

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        private static bool Finite(Vector3d value) => Finite(value.x) && Finite(value.y) && Finite(value.z);
    }
}
