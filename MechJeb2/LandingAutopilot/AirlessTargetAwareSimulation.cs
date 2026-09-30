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
        internal readonly double InitialMass, MaximumThrust, MinimumThrust;
        internal readonly double MaximumMassFlow, MinimumMassFlow;
        internal readonly double PolicyTerrainRadius, PolicyGravity, PolicyThrust;
        internal readonly bool HasV1ControlModel;
        internal readonly bool DecelerationAlreadyTriggered;
        internal readonly Vector3d InitialForward;
        internal readonly double MinimumCommandThrottle, MaximumCommandThrottle;
        internal readonly double ThrottleSmoothingSeconds, InitialAppliedThrottle;
        internal readonly double TouchdownSpeed, BottomOffset, PreviousTransThrottle;

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
            InitialMass = MaximumThrust = MinimumThrust = MaximumMassFlow = MinimumMassFlow = 0;
            PolicyTerrainRadius = PolicyGravity = PolicyThrust = 0;
            HasV1ControlModel = false;
            DecelerationAlreadyTriggered = false;
            InitialForward = Vector3d.zero;
            MinimumCommandThrottle = 0;
            MaximumCommandThrottle = 1;
            ThrottleSmoothingSeconds = 0;
            InitialAppliedThrottle = 0;
            TouchdownSpeed = 0.5;
            BottomOffset = 0;
            PreviousTransThrottle = 0;
        }

        internal AirlessTargetAwareSnapshot(AirlessTargetAwareSnapshot source,
            double initialMass, double maximumThrust, double minimumThrust,
            double maximumMassFlow, double minimumMassFlow,
            double policyTerrainRadius, double policyGravity, double policyThrust,
            bool decelerationAlreadyTriggered = false,
            Vector3d initialForward = default(Vector3d),
            double minimumCommandThrottle = 0, double maximumCommandThrottle = 1,
            double throttleSmoothingSeconds = 0, double initialAppliedThrottle = 0,
            double touchdownSpeed = 0.5, double bottomOffset = 0,
            double previousTransThrottle = 0)
        {
            InputUT = source.InputUT; EpochUT = source.EpochUT;
            BodyRadius = source.BodyRadius; BodyMu = source.BodyMu;
            BodyGeeASL = source.BodyGeeASL; RotationPeriod = source.RotationPeriod;
            TargetLatitude = source.TargetLatitude; TargetLongitude = source.TargetLongitude;
            TargetTerrainASL = source.TargetTerrainASL; DecelEndASL = source.DecelEndASL;
            MaximumThrustAcceleration = source.MaximumThrustAcceleration;
            Dt = source.Dt; MinDt = source.MinDt; MaxOrbits = source.MaxOrbits;
            MinimumTerrainASL = source.MinimumTerrainASL;
            MaximumTerrainASL = source.MaximumTerrainASL;
            Position = source.Position; Velocity = source.Velocity;
            AngularVelocity = source.AngularVelocity;
            Axis0 = source.Axis0; Axis90 = source.Axis90; AxisNorth = source.AxisNorth;
            InitialMass = initialMass; MaximumThrust = maximumThrust;
            MinimumThrust = minimumThrust; MaximumMassFlow = maximumMassFlow;
            MinimumMassFlow = minimumMassFlow;
            PolicyTerrainRadius = policyTerrainRadius;
            PolicyGravity = policyGravity; PolicyThrust = policyThrust;
            HasV1ControlModel = true;
            DecelerationAlreadyTriggered = decelerationAlreadyTriggered;
            InitialForward = initialForward;
            MinimumCommandThrottle = minimumCommandThrottle;
            MaximumCommandThrottle = maximumCommandThrottle;
            ThrottleSmoothingSeconds = throttleSmoothingSeconds;
            InitialAppliedThrottle = initialAppliedThrottle;
            TouchdownSpeed = touchdownSpeed;
            BottomOffset = bottomOffset;
            PreviousTransThrottle = previousTransThrottle;
        }

        internal AirlessTargetAwareSnapshot WithV1LandingTerrain(double terrainASL)
        {
            var source = new AirlessTargetAwareSnapshot(InputUT, EpochUT, BodyRadius,
                BodyMu, BodyGeeASL, RotationPeriod, TargetLatitude, TargetLongitude,
                TargetTerrainASL, terrainASL + 200, MaximumThrustAcceleration,
                Dt, MinDt, MaxOrbits, MinimumTerrainASL, MaximumTerrainASL,
                Position, Velocity, AngularVelocity, Axis0, Axis90, AxisNorth);
            return new AirlessTargetAwareSnapshot(source, InitialMass, MaximumThrust,
                MinimumThrust, MaximumMassFlow, MinimumMassFlow,
                BodyRadius + terrainASL + 200, PolicyGravity, PolicyThrust,
                DecelerationAlreadyTriggered, InitialForward,
                MinimumCommandThrottle, MaximumCommandThrottle,
                ThrottleSmoothingSeconds, InitialAppliedThrottle,
                TouchdownSpeed, BottomOffset, PreviousTransThrottle);
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
        internal readonly bool UsesV1ControlModel;
        internal readonly double EndMass;
        internal readonly Vector3d EndForward;
        internal readonly double EndAppliedThrottle;

        internal AirlessTargetAwareOutput(double brakeUT, List<AirlessTargetAwareState> coastSamples,
            List<AirlessTargetAwareState> trajectory, bool reachedHandoff,
            double endSurfaceSpeed, double virtualDeltaV, int steps,
            bool usesV1ControlModel = false, double endMass = double.NaN,
            Vector3d endForward = default(Vector3d), double endAppliedThrottle = 0)
        {
            BrakeUT = brakeUT;
            CoastSamples = coastSamples;
            Trajectory = trajectory;
            ReachedHandoff = reachedHandoff;
            EndSurfaceSpeed = endSurfaceSpeed;
            VirtualDeltaV = virtualDeltaV;
            Steps = steps;
            UsesV1ControlModel = usesV1ControlModel;
            EndMass = endMass;
            EndForward = endForward;
            EndAppliedThrottle = endAppliedThrottle;
        }

        internal AirlessTargetAwareState End => Trajectory[Trajectory.Count - 1];
    }

    // An optimistic lower bound for V1's horizontal-kill phase. It executes
    // V1's actual direction and hover-throttle policies, but assumes the
    // requested attitude is attained without delay. It is diagnostic only:
    // attitude response, terrain and FinalDescent remain unresolved.
    internal sealed class AirlessHorizontalKillBound
    {
        internal readonly List<AirlessTargetAwareState> Trajectory;
        internal readonly bool ReachedFinalDescent;
        internal readonly double EndHorizontalSpeed;
        internal readonly double DeltaV;
        internal readonly double EndMass;

        internal AirlessHorizontalKillBound(List<AirlessTargetAwareState> trajectory,
            bool reachedFinalDescent, double endHorizontalSpeed, double deltaV,
            double endMass)
        {
            Trajectory = trajectory;
            ReachedFinalDescent = reachedFinalDescent;
            EndHorizontalSpeed = endHorizontalSpeed;
            DeltaV = deltaV;
            EndMass = endMass;
        }
    }

    // A final-descent policy calculation on a fixed terrain height. The
    // planner must still resolve the entire generated path against live KSP
    // terrain, and the attitude/actuator assumptions preclude publication as
    // a certified landing without a controller-equivalence bound.
    internal sealed class AirlessFinalDescentBound
    {
        internal readonly List<AirlessTargetAwareState> Trajectory;
        internal readonly bool ReachedTerrain;
        internal readonly double ContactVerticalSpeed;
        internal readonly double ContactHorizontalSpeed;

        internal AirlessFinalDescentBound(List<AirlessTargetAwareState> trajectory,
            bool reachedTerrain, double contactVerticalSpeed,
            double contactHorizontalSpeed)
        {
            Trajectory = trajectory;
            ReachedTerrain = reachedTerrain;
            ContactVerticalSpeed = contactVerticalSpeed;
            ContactHorizontalSpeed = contactHorizontalSpeed;
        }
    }

    internal sealed class AirlessNominalTerminalOutput
    {
        internal readonly AirlessHorizontalKillBound HorizontalKill;
        internal readonly AirlessFinalDescentBound FinalDescent;
        internal readonly List<AirlessTargetAwareState> Trajectory;
        internal readonly double TerrainASL;
        internal readonly double HorizontalKillEndUT;

        internal AirlessNominalTerminalOutput(AirlessHorizontalKillBound horizontalKill,
            AirlessFinalDescentBound finalDescent, double terrainASL)
        {
            HorizontalKill = horizontalKill;
            FinalDescent = finalDescent;
            TerrainASL = terrainASL;
            HorizontalKillEndUT = horizontalKill.Trajectory[
                horizontalKill.Trajectory.Count - 1].UT;
            Trajectory = new List<AirlessTargetAwareState>(
                horizontalKill.Trajectory.Count +
                (finalDescent?.Trajectory.Count ?? 0));
            Trajectory.AddRange(horizontalKill.Trajectory);
            if (finalDescent != null)
                for (int i = 1; i < finalDescent.Trajectory.Count; i++)
                    Trajectory.Add(finalDescent.Trajectory[i]);
        }

        internal AirlessTargetAwareState End => Trajectory[Trajectory.Count - 1];
    }

    internal static class AirlessTargetAwareSimulation
    {
        private const int MaximumCoastSteps = 20000;

        internal static AirlessNominalTerminalOutput RunNominalTerminal(
            AirlessTargetAwareSnapshot snapshot, AirlessTargetAwareOutput braking,
            double localTerrainASL, double terrainSearchDepth = 0,
            double terminalTurnRate = NominalTerminalTurnRate)
        {
            if (braking == null || !braking.UsesV1ControlModel ||
                !braking.ReachedHandoff || !Finite(braking.EndMass) ||
                braking.EndMass <= 0)
                throw new ArgumentException("Incomplete V1 braking handoff");
            var horizontalKill = RunHorizontalKillContinuation(snapshot,
                braking.End, braking.EndMass, braking.EndForward,
                braking.EndAppliedThrottle, terminalTurnRate);
            var final = horizontalKill.ReachedFinalDescent ?
                RunIdealFinalDescentBound(snapshot, horizontalKill,
                    localTerrainASL, snapshot.TouchdownSpeed,
                    snapshot.PreviousTransThrottle, snapshot.BottomOffset, terrainSearchDepth) : null;
            return new AirlessNominalTerminalOutput(horizontalKill, final,
                localTerrainASL);
        }

        // Diagnostic worst-delay case for the terminal attitude transition.
        // V1 may thrust during this interval; treating it as unpowered is a
        // sensitivity check, not a replacement for its attitude controller.
        internal static AirlessHorizontalKillBound RunDelayedHorizontalKillBound(
            AirlessTargetAwareSnapshot snapshot, AirlessTargetAwareState handoff,
            double massAtHandoff, double unpoweredSeconds)
        {
            ValidateSnapshot(snapshot);
            if (!Finite(unpoweredSeconds) || unpoweredSeconds < 0 ||
                unpoweredSeconds > 60)
                throw new ArgumentException("Invalid terminal attitude delay");
            AirlessTargetAwareState state = handoff;
            var delay = new List<AirlessTargetAwareState> { state };
            double endUT = handoff.UT + unpoweredSeconds;
            while (state.UT < endUT - 1e-9)
            {
                state = RK4(state, Math.Min(snapshot.MinDt, endUT - state.UT),
                    snapshot.BodyMu);
                if (state.UT - delay[delay.Count - 1].UT >= 0.2 ||
                    state.UT >= endUT - 1e-9)
                    delay.Add(state);
            }
            Vector3d handoffSurface = handoff.Velocity -
                Vector3d.Cross(snapshot.AngularVelocity, handoff.Position);
            AirlessHorizontalKillBound kill = RunIdealHorizontalKillBound(snapshot,
                state, massAtHandoff);
            for (int i = 1; i < kill.Trajectory.Count; i++)
                delay.Add(kill.Trajectory[i]);
            return new AirlessHorizontalKillBound(delay,
                kill.ReachedFinalDescent, kill.EndHorizontalSpeed,
                kill.DeltaV, kill.EndMass);
        }

        internal static AirlessHorizontalKillBound RunIdealHorizontalKillBound(
            AirlessTargetAwareSnapshot snapshot, AirlessTargetAwareState handoff,
            double massAtHandoff)
            => RunHorizontalKillContinuation(snapshot, handoff, massAtHandoff,
                Vector3d.zero, 0, double.PositiveInfinity);

        // Observed average command-error reduction: Mun (62.25-5)/5.1
        // degrees/s; Minmus (47.97945766-4.33958439)/3.22 degrees/s.
        // These are conditional replay rates, not an attitude-controller law
        // or a certified actuator bound. Both paths receive terrain validation.
        internal const double SlowTerminalTurnRate = (62.25 - 5) / 5.1 * Math.PI / 180;
        internal const double FastTerminalTurnRate = (47.97945766 - 4.33958439) / 3.22 * Math.PI / 180;
        internal const double NominalTerminalTurnRate = (SlowTerminalTurnRate + FastTerminalTurnRate) / 2;

        internal static AirlessHorizontalKillBound RunHorizontalKillContinuation(
            AirlessTargetAwareSnapshot snapshot, AirlessTargetAwareState handoff,
            double massAtHandoff, Vector3d initialForward, double initialThrottle,
            double turnRate)
        {
            ValidateSnapshot(snapshot);
            if (!snapshot.HasV1ControlModel || massAtHandoff <= 0 ||
                snapshot.MaximumThrust <= 0 || snapshot.MaximumMassFlow < 0 ||
                !Finite(initialThrottle) || initialThrottle < 0 || initialThrottle > 1 ||
                double.IsNaN(turnRate) || turnRate <= 0)
                throw new ArgumentException("Incomplete V1 terminal snapshot");

            const double finalDescentHorizontalSpeed = 1.0;
            const int maximumSteps = 60000;
            double step = snapshot.MinDt;
            double mass = massAtHandoff;
            double appliedThrottle = initialThrottle;
            Vector3d forward = initialForward.normalized;
            double deltaV = 0;
            double lastSampleUT = handoff.UT;
            AirlessTargetAwareState state = handoff;
            var trajectory = new List<AirlessTargetAwareState> { state };
            for (int i = 0; i < maximumSteps; ++i)
            {
                Vector3d up = state.Position.normalized;
                Vector3d surface = state.Velocity -
                    Vector3d.Cross(snapshot.AngularVelocity, state.Position);
                double vertical = Vector3d.Dot(surface, up);
                double horizontal = Vector3d.Exclude(up, surface).magnitude;
                if (horizontal <= finalDescentHorizontalSpeed)
                {
                    if (trajectory[trajectory.Count - 1].UT != state.UT)
                        trajectory.Add(state);
                    return new AirlessHorizontalKillBound(trajectory, true,
                        horizontal, deltaV, mass);
                }
                if (state.Position.magnitude <=
                    snapshot.BodyRadius + snapshot.MinimumTerrainASL)
                    break;

                double gravity = snapshot.BodyMu / state.Position.sqrMagnitude;
                Vector3d commanded = V1LandingControlPolicy.HorizontalKillThrustDirection(
                    surface, up);
                if (forward.sqrMagnitude == 0) forward = commanded;
                Vector3d direction = forward;
                double throttle = V1LandingControlPolicy.HoverThrottle(vertical, gravity,
                    Vector3d.Dot(direction, up), snapshot.MaximumThrust / mass);
                throttle = Math.Min(snapshot.MaximumCommandThrottle,
                    Math.Max(snapshot.MinimumCommandThrottle, throttle));
                if (snapshot.ThrottleSmoothingSeconds > 0)
                    throttle = Math.Max(appliedThrottle - step / snapshot.ThrottleSmoothingSeconds,
                        Math.Min(appliedThrottle + step / snapshot.ThrottleSmoothingSeconds,
                            throttle));
                appliedThrottle = throttle;
                double thrust = snapshot.MinimumThrust + appliedThrottle *
                    (snapshot.MaximumThrust - snapshot.MinimumThrust);
                state = RK4WithThrust(state, step, snapshot.BodyMu,
                    direction * (thrust / mass));
                // Hover throttle uses the attained direction, as live V1 does;
                // thrust is retained while the craft turns toward its command.
                double angle = Math.Acos(Math.Max(-1, Math.Min(1,
                    Vector3d.Dot(forward, commanded))));
                if (angle <= turnRate * step || angle < 1e-9) forward = commanded;
                else
                {
                    double fraction = turnRate * step / angle;
                    forward = (Math.Sin((1 - fraction) * angle) * forward +
                        Math.Sin(fraction * angle) * commanded).normalized;
                }
                double massFlow = snapshot.MinimumMassFlow + appliedThrottle *
                    (snapshot.MaximumMassFlow - snapshot.MinimumMassFlow);
                mass = Math.Max(0.01 * massAtHandoff, mass - massFlow * step);
                deltaV += thrust / mass * step;
                if (!Finite(state.Position) || !Finite(state.Velocity))
                    throw new InvalidOperationException("Non-finite terminal bound");
                if (state.UT - lastSampleUT >= 1.0)
                {
                    trajectory.Add(state);
                    lastSampleUT = state.UT;
                }
            }
            if (trajectory[trajectory.Count - 1].UT != state.UT)
                trajectory.Add(state);
            Vector3d finalSurface = state.Velocity -
                Vector3d.Cross(snapshot.AngularVelocity, state.Position);
            return new AirlessHorizontalKillBound(trajectory, false,
                Vector3d.Exclude(state.Position.normalized, finalSurface).magnitude,
                deltaV, mass);
        }

        internal static AirlessFinalDescentBound RunIdealFinalDescentBound(
            AirlessTargetAwareSnapshot snapshot, AirlessHorizontalKillBound horizontalKill,
            double localTerrainASL, double touchdownSpeed, double priorThrottle,
            double bottomOffset = 0, double terrainSearchDepth = 0)
        {
            ValidateSnapshot(snapshot);
            if (horizontalKill == null || !horizontalKill.ReachedFinalDescent ||
                !Finite(localTerrainASL) || !Finite(touchdownSpeed) ||
                touchdownSpeed <= 0 || !Finite(priorThrottle) ||
                priorThrottle < 0 || priorThrottle > 1 || !Finite(bottomOffset) ||
                bottomOffset < 0 || !Finite(terrainSearchDepth) || terrainSearchDepth < 0)
                throw new ArgumentException("Incomplete V1 final-descent input");
            const int maximumSteps = 30000;
            double step = snapshot.MinDt;
            double mass = horizontalKill.EndMass;
            double previousError = 0;
            double integral = 0;
            double throttle = priorThrottle;
            AirlessTargetAwareState state =
                horizontalKill.Trajectory[horizontalKill.Trajectory.Count - 1];
            double lastSampleUT = state.UT;
            var trajectory = new List<AirlessTargetAwareState> { state };
            double radius = snapshot.BodyRadius + localTerrainASL + bottomOffset;
            // Keep a small path tail for the terrain resolver. This changes
            // only the simulation horizon, never the V1 speed/throttle law.
            // Actual first terrain contact still determines publication.
            double stopRadius = radius - terrainSearchDepth;
            for (int i = 0; i < maximumSteps; ++i)
            {
                Vector3d up = state.Position.normalized;
                Vector3d surface = state.Velocity -
                    Vector3d.Cross(snapshot.AngularVelocity, state.Position);
                double vertical = Vector3d.Dot(surface, up);
                double horizontal = Vector3d.Exclude(up, surface).magnitude;
                double clearance = state.Position.magnitude - radius;
                if (state.Position.magnitude <= stopRadius)
                {
                    if (trajectory[trajectory.Count - 1].UT != state.UT)
                        trajectory.Add(state);
                    return new AirlessFinalDescentBound(trajectory, true,
                        vertical, horizontal);
                }
                // This replay implements V1's final 300 m KEEP_VERTICAL
                // branch. Above that height V1 chooses among ascent,
                // attitude-gated retrograde, and KEEP_SURFACE branches. Do
                // not silently run the low-altitude law through that region.
                double controllerAltitude = Math.Min(clearance,
                    state.Position.magnitude - snapshot.BodyRadius);
                if (controllerAltitude > 300)
                    return new AirlessFinalDescentBound(trajectory, false,
                        vertical, horizontal);
                double maximumAcceleration = snapshot.MaximumThrust / mass;
                double localGravity = snapshot.BodyMu / state.Position.sqrMagnitude;
                double setpoint = V1LandingControlPolicy.FinalDescentSpeed(
                    controllerAltitude,
                    maximumAcceleration, localGravity, touchdownSpeed, true);
                Vector3d lateral = Vector3d.Exclude(up, surface);
                Vector3d direction = (-lateral + up *
                    Math.Max(Math.Abs(vertical), 20 * snapshot.BodyGeeASL)).normalized;
                double error = (setpoint - vertical) / maximumAcceleration;
                integral += error * step;
                double action = 0.05 * error + 0.000001 * integral +
                    0.05 * (error - previousError) / step;
                previousError = error;
                throttle = Math.Max(0, Math.Min(1, throttle + action));
                throttle = Math.Min(snapshot.MaximumCommandThrottle,
                    Math.Max(snapshot.MinimumCommandThrottle, throttle));
                double thrust = snapshot.MinimumThrust + throttle *
                    (snapshot.MaximumThrust - snapshot.MinimumThrust);
                AirlessTargetAwareState previous = state;
                state = RK4WithThrust(state, step, snapshot.BodyMu,
                    direction * (thrust / mass));
                double massFlow = snapshot.MinimumMassFlow + throttle *
                    (snapshot.MaximumMassFlow - snapshot.MinimumMassFlow);
                mass = Math.Max(0.01 * horizontalKill.EndMass, mass - massFlow * step);
                if (!Finite(state.Position) || !Finite(state.Velocity))
                    throw new InvalidOperationException("Non-finite final descent bound");
                if (state.Position.magnitude <= stopRadius)
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
                }
                if (state.UT - lastSampleUT >= (clearance < 30 ? 0.2 : 1.0) ||
                    state.Position.magnitude <= stopRadius)
                {
                    trajectory.Add(state);
                    lastSampleUT = state.UT;
                }
            }
            Vector3d endUp = state.Position.normalized;
            Vector3d endSurface = state.Velocity -
                Vector3d.Cross(snapshot.AngularVelocity, state.Position);
            if (trajectory[trajectory.Count - 1].UT != state.UT)
                trajectory.Add(state);
            return new AirlessFinalDescentBound(trajectory, false,
                Vector3d.Dot(endSurface, endUp),
                Vector3d.Exclude(endUp, endSurface).magnitude);
        }

        // One-way V1 baseline. The selected target is deliberately absent from
        // the timing decision. V1 enters DecelerationBurn at its speed-policy
        // trigger; its existing five-second guard releases at that same UT.
        internal static AirlessTargetAwareOutput RunNominalV1(AirlessTargetAwareSnapshot snapshot)
        {
            ValidateSnapshot(snapshot);
            if (!snapshot.HasV1ControlModel)
                throw new ArgumentException("V1 control inputs are required");
            if (snapshot.DecelerationAlreadyTriggered)
                return RunV1Control(snapshot, snapshot.InputUT, true);

            AirlessTargetAwareState state = new AirlessTargetAwareState(snapshot.Position,
                snapshot.Velocity, snapshot.InputUT);
            var coast = new List<AirlessTargetAwareState>();
            double period = 2 * Math.PI * Math.Sqrt(Math.Pow(snapshot.Position.magnitude, 3) /
                snapshot.BodyMu);
            for (int i = 0; i < MaximumCoastSteps && state.UT - snapshot.InputUT <= period; i++)
            {
                Vector3d surface = state.Velocity - Vector3d.Cross(snapshot.AngularVelocity,
                    state.Position);
                double allowed = V1AllowedSpeed(snapshot, state.Position, snapshot.InitialMass);
                if (Finite(allowed) && V1LandingControlPolicy.ShouldStartDeceleration(
                        surface.magnitude, allowed))
                    return RunV1Control(snapshot, state.UT + 5, true);

                if (state.Position.magnitude <= snapshot.BodyRadius + snapshot.MinimumTerrainASL)
                    break;
                state = RK4(state, 1, snapshot.BodyMu);
                if (state.Position.magnitude - snapshot.BodyRadius <= snapshot.MaximumTerrainASL)
                    coast.Add(state);
            }
            // The terrain resolver can still publish a first physical contact
            // from this coast; no powered endpoint has been invented.
            return new AirlessTargetAwareOutput(double.NaN, coast,
                new List<AirlessTargetAwareState> { state }, false,
                double.NaN, 0, coast.Count, true);
        }

        // Same airless gravity and speed-limiter integration as the existing
        // ReentrySimulation. Candidate 177's captured endpoint is the parity gate.
        internal static AirlessTargetAwareOutput Run(AirlessTargetAwareSnapshot snapshot, double brakeUT,
            bool normalizeHandoff = false)
        {
            ValidateSnapshot(snapshot);
            if (!Finite(brakeUT) || brakeUT < snapshot.InputUT)
                throw new ArgumentException("Invalid airless brake UT");
            if (snapshot.HasV1ControlModel)
                return RunV1Control(snapshot, brakeUT, normalizeHandoff);

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
            Vector3d initialSurface = state.Velocity -
                Vector3d.Cross(snapshot.AngularVelocity, state.Position);
            if (snapshot.InitialForward.sqrMagnitude > 0 &&
                (snapshot.DecelerationAlreadyTriggered ||
                 state.UT - snapshot.InputUT < 10) &&
                V1LandingControlPolicy.BrakeAttitudeGate(-1,
                    Vector3d.Dot(snapshot.InitialForward.normalized,
                        -initialSurface.normalized)))
                throw new InvalidOperationException("V1 attitude gate unresolved");
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

        // Forecast the commands DecelerationBurn actually requests. The brake
        // reference is a V1 input: its coast guard releases five seconds early.
        // The speed policy is the immutable policy installed by the live AP,
        // while thrust acceleration changes as the captured engine burns fuel.
        private static AirlessTargetAwareOutput RunV1Control(AirlessTargetAwareSnapshot snapshot,
            double brakeReferenceUT, bool normalizeHandoff)
        {
            if (snapshot.InitialMass <= 0 || snapshot.MaximumThrust <= 0 ||
                snapshot.MaximumMassFlow < 0 || snapshot.MinimumMassFlow < 0 ||
                snapshot.PolicyThrust <= snapshot.PolicyGravity ||
                snapshot.PolicyTerrainRadius <= snapshot.BodyRadius)
                throw new ArgumentException("Incomplete V1 control snapshot");
            if (!Finite(snapshot.MinimumCommandThrottle) ||
                !Finite(snapshot.MaximumCommandThrottle) ||
                !Finite(snapshot.ThrottleSmoothingSeconds) ||
                !Finite(snapshot.InitialAppliedThrottle) ||
                snapshot.MinimumCommandThrottle < 0 ||
                snapshot.MaximumCommandThrottle > 1 ||
                snapshot.MaximumCommandThrottle < snapshot.MinimumCommandThrottle ||
                snapshot.ThrottleSmoothingSeconds < 0 ||
                snapshot.InitialAppliedThrottle < 0 || snapshot.InitialAppliedThrottle > 1)
                throw new ArgumentException("Invalid V1 throttle snapshot");

            var coast = new List<AirlessTargetAwareState>();
            AirlessTargetAwareState state = new AirlessTargetAwareState(snapshot.Position,
                snapshot.Velocity, snapshot.InputUT);
            double releaseUT = Math.Max(snapshot.InputUT, brakeReferenceUT - 5.0);
            double stopRadius = snapshot.BodyRadius + snapshot.DecelEndASL + 5.0;
            bool policyTriggered = snapshot.DecelerationAlreadyTriggered;
            int coastSteps = 0;
            while (state.UT < releaseUT || !policyTriggered)
            {
                if (++coastSteps > MaximumCoastSteps)
                    throw new InvalidOperationException("V1 coast step limit exceeded");
                Vector3d surface = state.Velocity - Vector3d.Cross(snapshot.AngularVelocity,
                    state.Position);
                double allowed = V1AllowedSpeed(snapshot, state.Position, snapshot.InitialMass);
                if (Finite(allowed) && V1LandingControlPolicy.ShouldStartDeceleration(
                        surface.magnitude, allowed))
                    policyTriggered = true;
                if (state.UT >= releaseUT && policyTriggered)
                    break;
                if (state.Position.magnitude <= stopRadius ||
                    state.UT - snapshot.InputUT > 2 * Math.PI *
                    Math.Sqrt(Math.Pow(snapshot.Position.magnitude, 3) / snapshot.BodyMu))
                    return new AirlessTargetAwareOutput(brakeReferenceUT, coast,
                        new List<AirlessTargetAwareState> { state }, false,
                        surface.magnitude, 0, coastSteps, true);
                double step = Math.Min(1.0, Math.Max(0.02, releaseUT - state.UT));
                if (state.UT >= releaseUT) step = 0.2;
                state = RK4(state, step, snapshot.BodyMu);
                coast.Add(state);
            }

            // V1 withholds thrust while attitude is unresolved. A direct live
            // forecast cannot turn that interval into an immediate burn.
            if (snapshot.DecelerationAlreadyTriggered)
            {
                Vector3d liveSurface = state.Velocity - Vector3d.Cross(
                    snapshot.AngularVelocity, state.Position);
                if (snapshot.InitialForward.sqrMagnitude == 0 ||
                    V1LandingControlPolicy.BrakeAttitudeGate(
                        Vector3d.Dot(liveSurface, state.Position.normalized),
                        Vector3d.Dot(snapshot.InitialForward.normalized,
                            -liveSurface.normalized)))
                    throw new InvalidOperationException("V1 live attitude gate unresolved");
            }

            var trajectory = new List<AirlessTargetAwareState> { state };
            double mass = snapshot.InitialMass;
            double appliedThrottle = snapshot.DecelerationAlreadyTriggered ?
                snapshot.InitialAppliedThrottle : 0;
            Vector3d endForward = Vector3d.zero;
            double deltaV = 0;
            double maxTime = snapshot.MaxOrbits * 2 * Math.PI *
                Math.Sqrt(Math.Pow(state.Position.magnitude, 3) / snapshot.BodyMu);
            int steps = 0;
            bool reached = false;
            while (state.UT - brakeReferenceUT <= maxTime && steps++ < 50000)
            {
                AirlessTargetAwareState previous = state;
                double priorDeltaV = deltaV;
                double step = Math.Max(snapshot.MinDt, Math.Min(0.2, snapshot.Dt));
                Vector3d up = state.Position.normalized;
                Vector3d surface = state.Velocity - Vector3d.Cross(snapshot.AngularVelocity,
                    state.Position);
                double speed = surface.magnitude;
                if (speed < 1e-8) break;
                double allowed = V1AllowedSpeed(snapshot, state.Position, mass);
                Vector3d gravity = Gravity(state.Position, snapshot.BodyMu);
                // The next position can cross the 205 m handoff within this
                // step. V1 hands over there; querying its speed envelope
                // below the policy radius creates a false timeout.
                Vector3d nextPosition = state.Position + step * state.Velocity;
                if (nextPosition.magnitude < stopRadius)
                    nextPosition *= stopRadius / nextPosition.magnitude;
                double nextAllowed = V1AllowedSpeed(snapshot, nextPosition, mass);
                if (!Finite(allowed) || !Finite(nextAllowed)) break;
                double radialFraction = Math.Abs(Vector3d.Dot(surface.normalized, up));
                double minAccel = -gravity.magnitude * radialFraction;
                // The current braking tick has an observed thrust direction.
                // Later attitude is unresolved by this snapshot, so only that
                // first tick may use the copied physical forward vector.
                Vector3d thrustDirection = snapshot.DecelerationAlreadyTriggered &&
                    steps == 1 ? snapshot.InitialForward.normalized : -surface.normalized;
                endForward = thrustDirection;
                double alignment = Vector3d.Dot(thrustDirection, -surface.normalized);
                double maxAccel = snapshot.MaximumThrust / mass * alignment -
                    gravity.magnitude * radialFraction;
                double controlledSpeed = speed * Math.Sign(Vector3d.Dot(surface, up));
                double throttle = Vector3d.Dot(surface, up) > 0 ? 0 :
                    V1LandingControlPolicy.BrakingThrottle(controlledSpeed, -allowed,
                        -nextAllowed, step, minAccel, maxAccel);
                throttle = Math.Min(snapshot.MaximumCommandThrottle,
                    Math.Max(snapshot.MinimumCommandThrottle, throttle));
                if (snapshot.ThrottleSmoothingSeconds > 0)
                    throttle = Math.Max(appliedThrottle - step / snapshot.ThrottleSmoothingSeconds,
                        Math.Min(appliedThrottle + step / snapshot.ThrottleSmoothingSeconds,
                            throttle));
                appliedThrottle = throttle;
                double thrust = snapshot.MinimumThrust + appliedThrottle *
                    (snapshot.MaximumThrust - snapshot.MinimumThrust);
                Vector3d thrustAcceleration = thrustDirection * (thrust / mass);
                state = RK4WithThrust(state, step, snapshot.BodyMu, thrustAcceleration);
                double massFlow = snapshot.MinimumMassFlow + appliedThrottle *
                    (snapshot.MaximumMassFlow - snapshot.MinimumMassFlow);
                mass = Math.Max(0.01 * snapshot.InitialMass, mass - massFlow * step);
                deltaV += thrust / mass * step;
                reached = state.Position.magnitude <= stopRadius;
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
                    deltaV = priorDeltaV + fraction * (deltaV - priorDeltaV);
                }
                trajectory.Add(state);
                if (reached) break;
            }
            double endSpeed = (state.Velocity - Vector3d.Cross(snapshot.AngularVelocity,
                state.Position)).magnitude;
            return new AirlessTargetAwareOutput(brakeReferenceUT, coast, trajectory,
                reached, endSpeed, deltaV, steps, true, mass, endForward, appliedThrottle);
        }

        private static double V1AllowedSpeed(AirlessTargetAwareSnapshot snapshot,
            Vector3d position, double mass)
        {
            double availableThrust = snapshot.PolicyThrust * snapshot.InitialMass / mass;
            return V1LandingControlPolicy.SafeDescentMaximumSpeed(position.magnitude,
                snapshot.PolicyTerrainRadius, snapshot.PolicyGravity, availableThrust);
        }

        private static AirlessTargetAwareState RK4WithThrust(AirlessTargetAwareState state,
            double dt, double mu, Vector3d thrust)
        {
            Vector3d a1 = Gravity(state.Position, mu) + thrust;
            Vector3d v2 = state.Velocity + dt * 0.5 * a1;
            Vector3d a2 = Gravity(state.Position + dt * 0.5 * state.Velocity, mu) + thrust;
            Vector3d v3 = state.Velocity + dt * 0.5 * a2;
            Vector3d a3 = Gravity(state.Position + dt * 0.5 * v2, mu) + thrust;
            Vector3d v4 = state.Velocity + dt * a3;
            Vector3d a4 = Gravity(state.Position + dt * v3, mu) + thrust;
            return new AirlessTargetAwareState(state.Position + dt / 6 *
                (state.Velocity + 2 * v2 + 2 * v3 + v4),
                state.Velocity + dt / 6 * (a1 + 2 * a2 + 2 * a3 + a4), state.UT + dt);
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
            // Keep the pure worker geometry independent of MuUtils' KSP static
            // initializer so deterministic replay runs outside the game.
            longitude = ((longitude + 180) % 360 + 360) % 360 - 180;
            return new AbsoluteVector { Latitude = latitude, Longitude = longitude,
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
