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
        internal readonly double TouchdownSpeed, BottomOffset, TerminalBottomOffset, PreviousTransThrottle;
        internal readonly double SafeTouchdownSpeed;
        internal readonly double StateUT, DeliveryLeadSeconds, ObservedMassFlow;
        internal readonly Vector3d ObservedThrustAcceleration;
        internal readonly double TerminalControllerBottomOffset;
        // 0: future braking, 1: observed horizontal kill, 2: observed final descent.
        internal readonly int TerminalPhase;
        internal readonly double VerticalPidPreviousError, VerticalPidIntegral;
        internal readonly bool CoastToDeceleration;
        internal readonly double TerminalAngularAcceleration, TerminalAngularSpeed, TerminalStoppingTime;

        internal AirlessTargetAwareSnapshot(double inputUT, double epochUT, double bodyRadius,
            double bodyMu, double bodyGeeASL, double rotationPeriod, double targetLatitude,
            double targetLongitude, double targetTerrainASL, double decelEndASL,
            double maximumThrustAcceleration, double dt, double minDt, double maxOrbits,
            double minimumTerrainASL, double maximumTerrainASL, Vector3d position,
            Vector3d velocity, Vector3d angularVelocity, Vector3d axis0, Vector3d axis90,
            Vector3d axisNorth, double deliveryLeadSeconds = 0,
            Vector3d observedThrustAcceleration = default(Vector3d), double observedMassFlow = 0,
            double stateUT = double.NaN)
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
            SafeTouchdownSpeed = TouchdownSpeed;
            BottomOffset = 0;
            TerminalBottomOffset = 0;
            PreviousTransThrottle = 0;
            StateUT = double.IsNaN(stateUT) ? inputUT : stateUT;
            DeliveryLeadSeconds = deliveryLeadSeconds;
            ObservedThrustAcceleration = observedThrustAcceleration;
            ObservedMassFlow = observedMassFlow;
            TerminalControllerBottomOffset = 0;
            TerminalPhase = 0;
            VerticalPidPreviousError = VerticalPidIntegral = 0;
            CoastToDeceleration = false;
            TerminalAngularAcceleration = TerminalAngularSpeed = 0;
            TerminalStoppingTime = 2;
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
            double previousTransThrottle = 0, double safeTouchdownSpeed = double.NaN,
            double terminalBottomOffset = double.NaN,
            double terminalControllerBottomOffset = double.NaN,
            int terminalPhase = 0, double verticalPidPreviousError = 0,
            double verticalPidIntegral = 0, bool coastToDeceleration = false,
            double terminalAngularAcceleration = 0, double terminalAngularSpeed = 0,
            double terminalStoppingTime = 2)
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
            SafeTouchdownSpeed = double.IsNaN(safeTouchdownSpeed) ? touchdownSpeed : safeTouchdownSpeed;
            if (double.IsInfinity(SafeTouchdownSpeed) || SafeTouchdownSpeed <= 0)
                throw new ArgumentException("Invalid safe touchdown speed");
            BottomOffset = bottomOffset;
            TerminalBottomOffset = double.IsNaN(terminalBottomOffset) ? bottomOffset : terminalBottomOffset;
            if (double.IsInfinity(TerminalBottomOffset) || TerminalBottomOffset < 0)
                throw new ArgumentException("Invalid terminal contact offset");
            PreviousTransThrottle = previousTransThrottle;
            StateUT = source.StateUT;
            DeliveryLeadSeconds = source.DeliveryLeadSeconds;
            ObservedThrustAcceleration = source.ObservedThrustAcceleration;
            ObservedMassFlow = source.ObservedMassFlow;
            TerminalControllerBottomOffset = double.IsNaN(terminalControllerBottomOffset) ?
                TerminalBottomOffset : terminalControllerBottomOffset;
            TerminalPhase = terminalPhase;
            VerticalPidPreviousError = verticalPidPreviousError;
            VerticalPidIntegral = verticalPidIntegral;
            CoastToDeceleration = coastToDeceleration;
            TerminalAngularAcceleration = terminalAngularAcceleration;
            TerminalAngularSpeed = terminalAngularSpeed;
            TerminalStoppingTime = terminalStoppingTime;
            if (double.IsNaN(terminalAngularAcceleration) || double.IsInfinity(terminalAngularAcceleration) ||
                terminalAngularAcceleration < 0 || double.IsNaN(terminalAngularSpeed) ||
                double.IsInfinity(terminalAngularSpeed) || !(terminalStoppingTime > 0) ||
                double.IsInfinity(terminalStoppingTime))
                throw new ArgumentException("Invalid terminal attitude response snapshot");
        }

        internal AirlessTargetAwareSnapshot WithV1LandingTerrain(double terrainASL)
        {
            var source = new AirlessTargetAwareSnapshot(InputUT, EpochUT, BodyRadius,
                BodyMu, BodyGeeASL, RotationPeriod, TargetLatitude, TargetLongitude,
                TargetTerrainASL, terrainASL + 200, MaximumThrustAcceleration,
                Dt, MinDt, MaxOrbits, MinimumTerrainASL, MaximumTerrainASL,
                Position, Velocity, AngularVelocity, Axis0, Axis90, AxisNorth,
                DeliveryLeadSeconds, ObservedThrustAcceleration, ObservedMassFlow, StateUT);
            return new AirlessTargetAwareSnapshot(source, InitialMass, MaximumThrust,
                MinimumThrust, MaximumMassFlow, MinimumMassFlow,
                BodyRadius + terrainASL + 200, PolicyGravity, PolicyThrust,
                DecelerationAlreadyTriggered, InitialForward,
                MinimumCommandThrottle, MaximumCommandThrottle,
                ThrottleSmoothingSeconds, InitialAppliedThrottle,
                TouchdownSpeed, BottomOffset, PreviousTransThrottle, SafeTouchdownSpeed,
                TerminalBottomOffset, TerminalControllerBottomOffset, TerminalPhase,
                VerticalPidPreviousError, VerticalPidIntegral, CoastToDeceleration,
                TerminalAngularAcceleration, TerminalAngularSpeed, TerminalStoppingTime);
        }

        // A gimbal/attitude projection is a current measurement, not the engine's
        // sustained capability for the next several minutes. Keep installed
        // engine limits and mass flow; a real engine selection changes the vector.
        internal AirlessTargetAwareSnapshot WithEngineCapability(double thrust)
        {
            if (!(thrust > 0) || double.IsInfinity(thrust) || !(MaximumThrust > 0))
                throw new ArgumentException("Invalid engine capability");
            double ratio = thrust / MaximumThrust;
            var raw = new AirlessTargetAwareSnapshot(InputUT, EpochUT, BodyRadius, BodyMu,
                BodyGeeASL, RotationPeriod, TargetLatitude, TargetLongitude, TargetTerrainASL,
                DecelEndASL, MaximumThrustAcceleration * ratio, Dt, MinDt, MaxOrbits,
                MinimumTerrainASL, MaximumTerrainASL, Position, Velocity, AngularVelocity,
                Axis0, Axis90, AxisNorth, DeliveryLeadSeconds, ObservedThrustAcceleration,
                ObservedMassFlow, StateUT);
            return new AirlessTargetAwareSnapshot(raw, InitialMass, thrust, MinimumThrust,
                MaximumMassFlow, MinimumMassFlow, PolicyTerrainRadius, PolicyGravity,
                PolicyThrust * ratio, DecelerationAlreadyTriggered, InitialForward,
                MinimumCommandThrottle, MaximumCommandThrottle, ThrottleSmoothingSeconds,
                InitialAppliedThrottle, TouchdownSpeed, BottomOffset, PreviousTransThrottle,
                SafeTouchdownSpeed, TerminalBottomOffset, TerminalControllerBottomOffset,
                TerminalPhase, VerticalPidPreviousError, VerticalPidIntegral, CoastToDeceleration,
                TerminalAngularAcceleration, TerminalAngularSpeed, TerminalStoppingTime);
        }

        internal AirlessTargetAwareSnapshot WithAvailableThrustFraction(double fraction)
        {
            if (fraction <= 0 || fraction > 1 || double.IsNaN(fraction))
                throw new ArgumentException("Invalid available thrust fraction");
            var raw = new AirlessTargetAwareSnapshot(InputUT, EpochUT, BodyRadius, BodyMu,
                BodyGeeASL, RotationPeriod, TargetLatitude, TargetLongitude, TargetTerrainASL,
                DecelEndASL, MaximumThrustAcceleration * fraction, Dt, MinDt, MaxOrbits,
                MinimumTerrainASL, MaximumTerrainASL, Position, Velocity, AngularVelocity,
                Axis0, Axis90, AxisNorth, DeliveryLeadSeconds, ObservedThrustAcceleration,
                ObservedMassFlow, StateUT);
            return new AirlessTargetAwareSnapshot(raw, InitialMass, MaximumThrust * fraction,
                MinimumThrust * fraction, MaximumMassFlow * fraction, MinimumMassFlow * fraction,
                PolicyTerrainRadius, PolicyGravity, PolicyThrust * fraction,
                DecelerationAlreadyTriggered, InitialForward, MinimumCommandThrottle,
                MaximumCommandThrottle, ThrottleSmoothingSeconds, InitialAppliedThrottle,
                TouchdownSpeed, BottomOffset, PreviousTransThrottle, SafeTouchdownSpeed,
                TerminalBottomOffset, TerminalControllerBottomOffset, TerminalPhase,
                VerticalPidPreviousError, VerticalPidIntegral, CoastToDeceleration,
                TerminalAngularAcceleration, TerminalAngularSpeed, TerminalStoppingTime);
        }

        internal AirlessTargetAwareSnapshot WithState(AirlessTargetAwareState state)
        {
            var raw = new AirlessTargetAwareSnapshot(InputUT, EpochUT, BodyRadius, BodyMu,
                BodyGeeASL, RotationPeriod, TargetLatitude, TargetLongitude, TargetTerrainASL,
                DecelEndASL, MaximumThrustAcceleration, Dt, MinDt, MaxOrbits,
                MinimumTerrainASL, MaximumTerrainASL, state.Position, state.Velocity,
                AngularVelocity, Axis0, Axis90, AxisNorth, DeliveryLeadSeconds,
                ObservedThrustAcceleration, ObservedMassFlow, state.UT);
            return new AirlessTargetAwareSnapshot(raw, InitialMass, MaximumThrust, MinimumThrust,
                MaximumMassFlow, MinimumMassFlow, PolicyTerrainRadius, PolicyGravity, PolicyThrust,
                DecelerationAlreadyTriggered, InitialForward, MinimumCommandThrottle,
                MaximumCommandThrottle, ThrottleSmoothingSeconds, InitialAppliedThrottle,
                TouchdownSpeed, BottomOffset, PreviousTransThrottle, SafeTouchdownSpeed,
                TerminalBottomOffset, TerminalControllerBottomOffset, TerminalPhase,
                VerticalPidPreviousError, VerticalPidIntegral, CoastToDeceleration,
                TerminalAngularAcceleration, TerminalAngularSpeed, TerminalStoppingTime);
        }
    }

    internal readonly struct AirlessTargetAwareState
    {
        internal readonly Vector3d Position;
        internal readonly Vector3d Velocity;
        internal readonly double UT;
        internal readonly double PropulsiveDeltaV;

        internal AirlessTargetAwareState(Vector3d position, Vector3d velocity, double ut, double propulsiveDeltaV = double.NaN)
        {
            Position = position;
            Velocity = velocity;
            UT = ut;
            PropulsiveDeltaV = propulsiveDeltaV;
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
        internal readonly bool AttitudeUncertain;
        internal readonly AirlessTargetAwareState? FirstRequiredThrust;
        internal readonly bool SteeringReserveMaintained;

        internal AirlessTargetAwareOutput(double brakeUT, List<AirlessTargetAwareState> coastSamples,
            List<AirlessTargetAwareState> trajectory, bool reachedHandoff,
            double endSurfaceSpeed, double virtualDeltaV, int steps,
            bool usesV1ControlModel = false, double endMass = double.NaN,
            Vector3d endForward = default(Vector3d), double endAppliedThrottle = 0,
            bool attitudeUncertain = false, AirlessTargetAwareState? firstRequiredThrust = null,
            bool steeringReserveMaintained = true)
        {
            BrakeUT = brakeUT;
            AttitudeUncertain = attitudeUncertain;
            FirstRequiredThrust = firstRequiredThrust;
            SteeringReserveMaintained=steeringReserveMaintained;
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

        internal AirlessTargetAwareState BrakeReferenceState()
        {
            // BrakeUT is the controller reference, not necessarily ignition or
            // the first displayed path sample. Keep its marker at the same UT.
            AirlessTargetAwareState previous = CoastSamples.Count > 0 ? CoastSamples[0] : Trajectory[0];
            var samples = BrakeUT < Trajectory[0].UT && CoastSamples.Count > 0 ? CoastSamples : Trajectory;
            foreach (AirlessTargetAwareState next in samples)
            {
                if (next.UT >= BrakeUT)
                {
                    double f = next.UT > previous.UT ?
                        Math.Max(0, Math.Min(1, (BrakeUT - previous.UT) / (next.UT - previous.UT))) : 0;
                    return new AirlessTargetAwareState(previous.Position + f * (next.Position - previous.Position),
                        previous.Velocity + f * (next.Velocity - previous.Velocity), BrakeUT);
                }
                previous = next;
            }
            return previous;
        }
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

        // Only the observed ongoing burn is extrapolated across delivery latency.
        // InputUT/EpochUT remain the observation lineage; StateUT names the
        // effective state. This runs on the predictor worker, never with KSP APIs.
        internal static AirlessTargetAwareSnapshot AtDeliveryState(AirlessTargetAwareSnapshot snapshot)
        {
            double endUT = snapshot.InputUT + snapshot.DeliveryLeadSeconds;
            if (!Finite(snapshot.DeliveryLeadSeconds) || snapshot.DeliveryLeadSeconds > 0.5 ||
                snapshot.DeliveryLeadSeconds < 0 || !Finite(snapshot.ObservedThrustAcceleration) ||
                !Finite(snapshot.ObservedMassFlow) || snapshot.ObservedMassFlow < 0)
                throw new ArgumentException("Invalid observed-burn delivery lead");
            if (endUT <= snapshot.StateUT) return snapshot;
            ValidateSnapshot(snapshot);
            if (!snapshot.HasV1ControlModel || !Finite(snapshot.InitialMass) || snapshot.InitialMass <= 0)
                throw new ArgumentException("Observed burn lacks a mass/thrust snapshot");
            var state = new AirlessTargetAwareState(snapshot.Position, snapshot.Velocity, snapshot.StateUT);
            double mass = snapshot.InitialMass;
            while (state.UT < endUT - 1e-9)
            {
                double step = Math.Min(snapshot.MinDt, endUT - state.UT);
                state = RK4WithThrust(state, step, snapshot.BodyMu,
                    snapshot.ObservedThrustAcceleration * (snapshot.InitialMass / mass));
                mass = Math.Max(snapshot.InitialMass * 0.01, mass - snapshot.ObservedMassFlow * step);
            }
            double ratio = snapshot.InitialMass / mass;
            var raw = new AirlessTargetAwareSnapshot(snapshot.InputUT, snapshot.EpochUT,
                snapshot.BodyRadius, snapshot.BodyMu, snapshot.BodyGeeASL, snapshot.RotationPeriod,
                snapshot.TargetLatitude, snapshot.TargetLongitude, snapshot.TargetTerrainASL,
                snapshot.DecelEndASL, snapshot.MaximumThrustAcceleration * ratio,
                snapshot.Dt, snapshot.MinDt, snapshot.MaxOrbits, snapshot.MinimumTerrainASL,
                snapshot.MaximumTerrainASL, state.Position, state.Velocity, snapshot.AngularVelocity,
                snapshot.Axis0, snapshot.Axis90, snapshot.AxisNorth, snapshot.DeliveryLeadSeconds,
                snapshot.ObservedThrustAcceleration, snapshot.ObservedMassFlow, state.UT);
            return new AirlessTargetAwareSnapshot(raw, mass, snapshot.MaximumThrust,
                snapshot.MinimumThrust, snapshot.MaximumMassFlow, snapshot.MinimumMassFlow,
                snapshot.PolicyTerrainRadius, snapshot.PolicyGravity, snapshot.PolicyThrust * ratio,
                snapshot.DecelerationAlreadyTriggered, snapshot.InitialForward,
                snapshot.MinimumCommandThrottle, snapshot.MaximumCommandThrottle,
                snapshot.ThrottleSmoothingSeconds, snapshot.InitialAppliedThrottle,
                snapshot.TouchdownSpeed, snapshot.BottomOffset, snapshot.PreviousTransThrottle,
                snapshot.SafeTouchdownSpeed, snapshot.TerminalBottomOffset, snapshot.TerminalControllerBottomOffset,
                snapshot.TerminalPhase, snapshot.VerticalPidPreviousError, snapshot.VerticalPidIntegral,
                snapshot.CoastToDeceleration, snapshot.TerminalAngularAcceleration,
                snapshot.TerminalAngularSpeed, snapshot.TerminalStoppingTime);
        }

        internal static AirlessTargetAwareOutput RunObservedTerminal(AirlessTargetAwareSnapshot snapshot,
            double executedBrakeReferenceUT)
        {
            ValidateSnapshot(snapshot);
            if (snapshot.TerminalPhase < 1 || snapshot.TerminalPhase > 2)
                throw new ArgumentException("No observed terminal phase");
            var state = new AirlessTargetAwareState(snapshot.Position, snapshot.Velocity, snapshot.StateUT, 0);
            return new AirlessTargetAwareOutput(Finite(executedBrakeReferenceUT) ? executedBrakeReferenceUT :
                snapshot.StateUT, new List<AirlessTargetAwareState>(),
                new List<AirlessTargetAwareState> { state }, true,
                (state.Velocity - Vector3d.Cross(snapshot.AngularVelocity, state.Position)).magnitude,
                0, 0, true, snapshot.InitialMass, snapshot.InitialForward, snapshot.InitialAppliedThrottle);
        }

        internal static AirlessNominalTerminalOutput RunNominalTerminal(
            AirlessTargetAwareSnapshot snapshot, AirlessTargetAwareOutput braking,
            double localTerrainASL, double terrainSearchDepth = 0,
            double terminalTurnRate = NominalTerminalTurnRate)
        {
            snapshot = AtDeliveryState(snapshot);
            if (braking == null || !braking.UsesV1ControlModel ||
                !braking.ReachedHandoff || !Finite(braking.EndMass) ||
                braking.EndMass <= 0)
                throw new ArgumentException("Incomplete V1 braking handoff");
            var horizontalKill = snapshot.TerminalPhase == 2 ?
                new AirlessHorizontalKillBound(new List<AirlessTargetAwareState> { braking.End }, true,
                    Vector3d.Exclude(braking.End.Position.normalized, braking.End.Velocity -
                        Vector3d.Cross(snapshot.AngularVelocity, braking.End.Position)).magnitude,
                    0, braking.EndMass) : RunHorizontalKillContinuation(snapshot,
                braking.End, braking.EndMass, braking.EndForward,
                braking.EndAppliedThrottle, terminalTurnRate);
            var final = horizontalKill.ReachedFinalDescent ?
                RunIdealFinalDescentBound(snapshot, horizontalKill,
                    localTerrainASL, snapshot.TouchdownSpeed,
                    snapshot.PreviousTransThrottle, snapshot.TerminalBottomOffset, terrainSearchDepth,
                    snapshot.TerminalControllerBottomOffset, terminalTurnRate) : null;
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
            double angularSpeed = snapshot.TerminalPhase == 1 ? snapshot.TerminalAngularSpeed : 0;
            double rateIntegral = 0, torqueIntegral = 0;
            Vector3d previousTurnAxis = Vector3d.zero;
            double deltaV = 0;
            double lastSampleUT = handoff.UT;
            AirlessTargetAwareState state = new AirlessTargetAwareState(handoff.Position, handoff.Velocity, handoff.UT, 0);
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
                double turn = turnRate * step;
                if (snapshot.TerminalAngularAcceleration > 0 && Finite(turnRate))
                {
                    Vector3d axis = Vector3d.Cross(forward, commanded).normalized;
                    if (axis.sqrMagnitude == 0) axis = previousTurnAxis;
                    if (previousTurnAxis.sqrMagnitude > 0)
                    {
                        double projection = Vector3d.Dot(previousTurnAxis, axis);
                        angularSpeed *= projection;
                        rateIntegral *= projection;
                        torqueIntegral *= projection;
                    }
                    previousTurnAxis = axis;
                    double alpha = snapshot.TerminalAngularAcceleration *
                        (turnRate / NominalTerminalTurnRate);
                    // Scalar projection of HybridController's inertia-compensated
                    // rate PI and TorquePI. Authority comes from this vessel's
                    // torque/inertia snapshot, never from the selected target.
                    double error = angle - angularSpeed * Math.Abs(angularSpeed) / (2 * alpha);
                    double targetRate = ClampedPI(error, 1, .1,
                        snapshot.TerminalStoppingTime * alpha, step, ref rateIntegral);
                    double acceleration = ClampedPI(targetRate - angularSpeed, 4, 4,
                        alpha, step, ref torqueIntegral);
                    double nextRate = angularSpeed + acceleration * step;
                    turn = .5 * (angularSpeed + nextRate) * step;
                    angularSpeed = nextRate;
                    // Preserve angular momentum through the requested direction;
                    // snapping to it suppresses the real controller's overshoot.
                    if (axis.sqrMagnitude > 0)
                        forward = (forward * Math.Cos(turn) +
                            Vector3d.Cross(axis, forward) * Math.Sin(turn)).normalized;
                }
                else if (angle <= turn || angle < 1e-9) forward = commanded;
                else
                {
                    double fraction = turn / angle;
                    forward = (Math.Sin((1 - fraction) * angle) * forward +
                        Math.Sin(fraction * angle) * commanded).normalized;
                }
                double massFlow = snapshot.MinimumMassFlow + appliedThrottle *
                    (snapshot.MaximumMassFlow - snapshot.MinimumMassFlow);
                mass = Math.Max(0.01 * massAtHandoff, mass - massFlow * step);
                deltaV += thrust / mass * step;
                state = new AirlessTargetAwareState(state.Position, state.Velocity, state.UT, deltaV);
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

        private static double ClampedPI(double error, double kp, double ki,
            double limit, double dt, ref double integral)
        {
            double p = kp * error;
            integral += ki * error * dt;
            double value = Math.Max(-limit, Math.Min(limit, p + integral));
            integral = value - Math.Max(-limit, Math.Min(limit, p));
            return value;
        }

        internal static AirlessFinalDescentBound RunIdealFinalDescentBound(
            AirlessTargetAwareSnapshot snapshot, AirlessHorizontalKillBound horizontalKill,
            double localTerrainASL, double touchdownSpeed, double priorThrottle,
            double bottomOffset = 0, double terrainSearchDepth = 0,
            double controllerBottomOffset = double.NaN,
            double terminalTurnRate = NominalTerminalTurnRate)
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
            double previousError = snapshot.TerminalPhase == 2 ? snapshot.VerticalPidPreviousError : 0;
            double integral = snapshot.TerminalPhase == 2 ? snapshot.VerticalPidIntegral : 0;
            double throttle = priorThrottle;
            AirlessTargetAwareState state =
                horizontalKill.Trajectory[horizontalKill.Trajectory.Count - 1];
            // V1 requests surface-up when leaving horizontal kill. Subsequent
            // upper-descent retrograde turns use the same bounded response as
            // the preceding terminal phase; the lower branch is unchanged.
            Vector3d forward = snapshot.TerminalPhase == 2 ?
                snapshot.InitialForward.normalized : state.Position.normalized;
            double lastSampleUT = state.UT;
            var trajectory = new List<AirlessTargetAwareState> { state };
            double radius = snapshot.BodyRadius + localTerrainASL + bottomOffset;
            if (double.IsNaN(controllerBottomOffset)) controllerBottomOffset = bottomOffset;
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
                // Contact support and V1's max-extent bottom-altitude sphere
                // are different inputs. Do not slow at an invented contact
                // bound as though V1 used it as its controller altitude.
                double controllerAltitude = Math.Min(state.Position.magnitude - snapshot.BodyRadius -
                    localTerrainASL - controllerBottomOffset,
                    state.Position.magnitude - snapshot.BodyRadius);
                double maximumAcceleration = snapshot.MaximumThrust / mass;
                double localGravity = snapshot.BodyMu / state.Position.sqrMagnitude;
                double setpoint = V1LandingControlPolicy.FinalDescentSpeed(
                    controllerAltitude,
                    maximumAcceleration, localGravity, touchdownSpeed, true);
                Vector3d lateral = Vector3d.Exclude(up, surface);
                Vector3d direction = (-lateral + up *
                    Math.Max(Math.Abs(vertical), 20 * snapshot.BodyGeeASL)).normalized;
                double error = (setpoint - vertical) / maximumAcceleration;
                bool direct = false;
                bool upperDescent = controllerAltitude > 300 && maximumAcceleration >= localGravity;
                if (upperDescent)
                {
                    // Mirror FinalDescent's existing >300 m branches. Terrain
                    // remains a resolved local model, subsequently checked
                    // against KSP's sampled path, never a fixed body surface.
                    bool ascending = surface.magnitude > 5 && Vector3d.Angle(surface, up) < 80;
                    direction = ascending ? up : -surface.normalized;
                    direct = ascending || surface.magnitude > 5 && Vector3d.Angle(forward, direction) > 45;
                    setpoint = direct ? snapshot.MinimumCommandThrottle * 100 :
                        new GravityTurnDescentSpeedPolicy(snapshot.BodyRadius + localTerrainASL,
                            snapshot.BodyGeeASL * 9.81, maximumAcceleration).MaxAllowedSpeed(state.Position, surface);
                    error = (setpoint - surface.magnitude) / maximumAcceleration;
                    if (Vector3d.Dot(forward, surface) < 0) error *= -1;
                }
                integral += error * step;
                double action = 0.05 * error + 0.000001 * integral +
                    0.05 * (error - previousError) / step;
                previousError = error;
                throttle = direct ? snapshot.MinimumCommandThrottle : Math.Max(0, Math.Min(1, throttle + action));
                // FinalDescent's DIRECT branch requests the configured minimum.
                // KEEP_VERTICAL/KEEP_SURFACE assign the speed PID's clamped
                // command directly; RequestActiveThrottle's minimum floor is
                // not part of that path. Applying it here can turn a high-TWR
                // low-gravity descent into a fictitious sustained ascent.
                throttle = Math.Min(snapshot.MaximumCommandThrottle, Math.Max(0, throttle));
                double thrust = snapshot.MinimumThrust + throttle *
                    (snapshot.MaximumThrust - snapshot.MinimumThrust);
                Vector3d appliedDirection = direction;
                if (upperDescent)
                {
                    double angle = Math.Acos(Math.Max(-1, Math.Min(1, Vector3d.Dot(forward, direction))));
                    double turn = terminalTurnRate * step;
                    if (angle <= turn || angle < 1e-9) forward = direction;
                    else
                    {
                        double fraction = turn / angle;
                        forward = (Math.Sin((1-fraction)*angle)*forward + Math.Sin(fraction*angle)*direction).normalized;
                    }
                    appliedDirection = forward;
                }
                else forward = direction;
                AirlessTargetAwareState previous = state;
                double previousImpulse = Finite(state.PropulsiveDeltaV) ? state.PropulsiveDeltaV : horizontalKill.DeltaV;
                state = RK4WithThrust(state, step, snapshot.BodyMu,
                    appliedDirection * (thrust / mass));
                double massFlow = snapshot.MinimumMassFlow + throttle *
                    (snapshot.MaximumMassFlow - snapshot.MinimumMassFlow);
                mass = Math.Max(0.01 * horizontalKill.EndMass, mass - massFlow * step);
                state = new AirlessTargetAwareState(state.Position, state.Velocity, state.UT, previousImpulse + thrust / mass * step);
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
                        previous.UT + fraction * (state.UT - previous.UT),
                        previousImpulse + fraction * (state.PropulsiveDeltaV - previousImpulse));
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
            if (snapshot.CoastToDeceleration)
                return RunCoastToDeceleration(snapshot, snapshot.InputUT);
            snapshot = AtDeliveryState(snapshot);
            ValidateSnapshot(snapshot);
            if (!snapshot.HasV1ControlModel)
                throw new ArgumentException("V1 control inputs are required");
            if (snapshot.DecelerationAlreadyTriggered)
                return RunV1Control(snapshot, snapshot.InputUT, true);

            AirlessTargetAwareState state = new AirlessTargetAwareState(snapshot.Position,
                snapshot.Velocity, snapshot.StateUT);
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

        internal static AirlessTargetAwareOutput RunCoastForecast(AirlessTargetAwareSnapshot snapshot,
            double brakeReferenceUT)
        {
            if (snapshot.CoastToDeceleration)
                return RunCoastToDeceleration(snapshot, brakeReferenceUT);
            // The fixed reference belongs to the current V1 sequence. Terrain
            // validates this freshly propagated coast before publication.
            return Finite(brakeReferenceUT) ? Run(snapshot,
                Math.Max(snapshot.InputUT + snapshot.DeliveryLeadSeconds, brakeReferenceUT), true) :
                RunNominalV1(snapshot);
        }

        // CoastToDeceleration keeps thrust off until its 0.9 envelope trigger.
        // Its speed trigger and DecelerationBurn's reference clock are separate.
        // Preserve every propagated coast point for first-intersection checking.
        private static AirlessTargetAwareOutput RunCoastToDeceleration(
            AirlessTargetAwareSnapshot snapshot, double referenceUT)
        {
            snapshot = AtDeliveryState(snapshot);
            ValidateSnapshot(snapshot);
            var state = new AirlessTargetAwareState(snapshot.Position, snapshot.Velocity, snapshot.StateUT);
            var coast = new List<AirlessTargetAwareState>();
            double period = 2 * Math.PI * Math.Sqrt(Math.Pow(snapshot.Position.magnitude, 3) / snapshot.BodyMu);
            for (int i = 0; i < MaximumCoastSteps && state.UT - snapshot.StateUT <= period; i++)
            {
                double speed = (state.Velocity - Vector3d.Cross(snapshot.AngularVelocity, state.Position)).magnitude;
                double allowed = V1AllowedSpeed(snapshot, state.Position, snapshot.InitialMass);
                if (state.Position.magnitude < snapshot.BodyRadius + snapshot.DecelEndASL + 5 ||
                    Finite(allowed) && V1LandingControlPolicy.ShouldStartDeceleration(speed, allowed))
                {
                    var powered = RunV1Control(snapshot.WithState(state),
                        Finite(referenceUT) ? referenceUT : state.UT + 5, true);
                    coast.AddRange(powered.CoastSamples);
                    return new AirlessTargetAwareOutput(powered.BrakeUT, coast, powered.Trajectory,
                        powered.ReachedHandoff, powered.EndSurfaceSpeed, powered.VirtualDeltaV,
                        i + powered.Steps, true, powered.EndMass, powered.EndForward,
                        powered.EndAppliedThrottle, powered.AttitudeUncertain, powered.FirstRequiredThrust);
                }
                if (state.Position.magnitude <= snapshot.BodyRadius + snapshot.MinimumTerrainASL) break;
                coast.Add(state);
                state = RK4(state, 1, snapshot.BodyMu);
            }
            return new AirlessTargetAwareOutput(double.NaN, coast,
                new List<AirlessTargetAwareState> { state }, false,
                (state.Velocity - Vector3d.Cross(snapshot.AngularVelocity, state.Position)).magnitude,
                0, coast.Count, true, snapshot.InitialMass);
        }

        // Same airless gravity and speed-limiter integration as the existing
        // ReentrySimulation. Candidate 177's captured endpoint is the parity gate.
        internal static AirlessTargetAwareOutput Run(AirlessTargetAwareSnapshot snapshot, double brakeUT,
            bool normalizeHandoff = false)
        {
            snapshot = AtDeliveryState(snapshot);
            ValidateSnapshot(snapshot);
            if (!Finite(brakeUT) || brakeUT < snapshot.InputUT)
                throw new ArgumentException("Invalid airless brake UT");
            if (snapshot.HasV1ControlModel)
                return RunV1Control(snapshot, brakeUT, normalizeHandoff);

            var coastSamples = new List<AirlessTargetAwareState>();
            AirlessTargetAwareState state = new AirlessTargetAwareState(snapshot.Position,
                snapshot.Velocity, snapshot.StateUT);
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
        internal static AirlessTargetAwareOutput RunBrakingSteeringProbe(AirlessTargetAwareSnapshot snapshot,
            Vector3d steering,double endUT,double authorityScale=1) =>
            RunV1Control(snapshot,snapshot.InputUT,true,steering,endUT,authorityScale);

        private static AirlessTargetAwareOutput RunV1Control(AirlessTargetAwareSnapshot snapshot,
            double brakeReferenceUT, bool normalizeHandoff, Vector3d steering=default(Vector3d),
            double steeringEndUT=double.NegativeInfinity,double authorityScale=1)
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
                snapshot.Velocity, snapshot.StateUT);
            double releaseUT = Math.Max(snapshot.StateUT, brakeReferenceUT - 5.0);
            double stopRadius = snapshot.BodyRadius + snapshot.DecelEndASL + 5.0;
            int coastSteps = 0;
            // DecelerationBurn releases at reference minus five seconds. Its
            // speed envelope determines throttle, not a second coast trigger.
            // In particular minimum throttle applies from this release even
            // when the closed-loop requested throttle is zero.
            while (state.UT < releaseUT)
            {
                if (++coastSteps > MaximumCoastSteps)
                    throw new InvalidOperationException("V1 coast step limit exceeded");
                Vector3d surface = state.Velocity - Vector3d.Cross(snapshot.AngularVelocity,
                    state.Position);
                if (state.Position.magnitude <= stopRadius ||
                    state.UT - snapshot.InputUT > 2 * Math.PI *
                    Math.Sqrt(Math.Pow(snapshot.Position.magnitude, 3) / snapshot.BodyMu))
                    return new AirlessTargetAwareOutput(brakeReferenceUT, coast,
                        new List<AirlessTargetAwareState> { state }, false,
                        surface.magnitude, 0, coastSteps, true);
                double step = Math.Min(1.0, releaseUT - state.UT);
                state = RK4(state, step, snapshot.BodyMu);
                coast.Add(state);
            }

            // V1 withholds thrust while attitude is unresolved. A direct live
            // forecast cannot turn that interval into an immediate burn.
            // Upward motion alone is not unresolved attitude: integrate V1's
            // zero-throttle ascent below, retaining its future thrust deadline.
            if (snapshot.DecelerationAlreadyTriggered ||
                releaseUT == snapshot.InputUT + snapshot.DeliveryLeadSeconds && snapshot.InitialForward.sqrMagnitude > 0)
            {
                Vector3d liveSurface = state.Velocity - Vector3d.Cross(
                    snapshot.AngularVelocity, state.Position);
                if (snapshot.InitialForward.sqrMagnitude == 0 ||
                    Vector3d.Dot(snapshot.InitialForward.normalized,
                        -liveSurface.normalized) < 0.75)
                    return new AirlessTargetAwareOutput(brakeReferenceUT, coast,
                        new List<AirlessTargetAwareState> { state }, false,
                        liveSurface.magnitude, 0, coastSteps, true, snapshot.InitialMass,
                        snapshot.InitialForward, snapshot.InitialAppliedThrottle, true);
            }

            state = new AirlessTargetAwareState(state.Position, state.Velocity, state.UT, 0);
            var trajectory = new List<AirlessTargetAwareState> { state };
            double mass = snapshot.InitialMass;
            double appliedThrottle = snapshot.DecelerationAlreadyTriggered ?
                snapshot.InitialAppliedThrottle : 0;
            Vector3d endForward = Vector3d.zero;
            double deltaV = 0;
            double maxTime = snapshot.MaxOrbits * 2 * Math.PI *
                Math.Sqrt(Math.Pow(state.Position.magnitude, 3) / snapshot.BodyMu);
            // Waiting for the coast speed trigger is not powered simulation
            // time. An early search reference must not consume the braking
            // horizon before the actual braking trajectory begins.
            double brakingStartUT = state.UT;
            int steps = 0;
            bool reached = false;
            AirlessTargetAwareState? firstRequiredThrust = null;
            Vector3d probeForward=snapshot.InitialForward.normalized,probeAxis=Vector3d.zero;
            if(!snapshot.DecelerationAlreadyTriggered)
                probeForward=-(state.Velocity-Vector3d.Cross(snapshot.AngularVelocity,state.Position)).normalized;
            double probeOmega=0,probeRateIntegral=0,probeTorqueIntegral=0;
            bool steeringReserveMaintained=true;
            while (state.UT - brakingStartUT <= maxTime && steps++ < 50000)
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
                Vector3d desiredDirection=state.UT<steeringEndUT ?
                    (-surface.normalized+steering).normalized : -surface.normalized;
                Vector3d thrustDirection = (snapshot.DecelerationAlreadyTriggered ||
                    brakingStartUT == snapshot.InputUT + snapshot.DeliveryLeadSeconds && snapshot.InitialForward.sqrMagnitude > 0) &&
                    steps == 1 ? snapshot.InitialForward.normalized : desiredDirection;
                if(snapshot.TerminalAngularAcceleration>0)
                    thrustDirection=probeForward;
                endForward = thrustDirection;
                double alignment = Vector3d.Dot(thrustDirection, -surface.normalized);
                double maxAccel = snapshot.MaximumThrust / mass * alignment -
                    gravity.magnitude * radialFraction;
                double controlledSpeed = speed * Math.Sign(Vector3d.Dot(surface, up));
                double throttle = Vector3d.Dot(surface, up) > 0 ||
                    snapshot.TerminalAngularAcceleration>0 && Vector3d.Dot(thrustDirection,desiredDirection)<.75 ? 0 :
                    V1LandingControlPolicy.BrakingThrottle(controlledSpeed, -allowed,
                        -nextAllowed, step, minAccel, maxAccel);
                if(steering.sqrMagnitude>0 && state.UT<steeringEndUT &&
                    throttle>.9*snapshot.MaximumCommandThrottle)
                    steeringReserveMaintained=false;
                throttle = Math.Min(snapshot.MaximumCommandThrottle,
                    Math.Max(snapshot.MinimumCommandThrottle, throttle));
                if (snapshot.ThrottleSmoothingSeconds > 0)
                    throttle = Math.Max(appliedThrottle - step / snapshot.ThrottleSmoothingSeconds,
                        Math.Min(appliedThrottle + step / snapshot.ThrottleSmoothingSeconds,
                            throttle));
                appliedThrottle = throttle;
                double thrust = snapshot.MinimumThrust + appliedThrottle *
                    (snapshot.MaximumThrust - snapshot.MinimumThrust);
                if (thrust > 0 && !firstRequiredThrust.HasValue) firstRequiredThrust = state;
                Vector3d thrustAcceleration = thrustDirection * (thrust / mass);
                state = RK4WithThrust(state, step, snapshot.BodyMu, thrustAcceleration);
                double massFlow = snapshot.MinimumMassFlow + appliedThrottle *
                    (snapshot.MaximumMassFlow - snapshot.MinimumMassFlow);
                mass = Math.Max(0.01 * snapshot.InitialMass, mass - massFlow * step);
                deltaV += thrust / mass * step;
                state = new AirlessTargetAwareState(state.Position, state.Velocity, state.UT, deltaV);
                if(snapshot.TerminalAngularAcceleration>0)
                {
                    Vector3d axis=Vector3d.Cross(probeForward,desiredDirection).normalized;
                    if(axis.sqrMagnitude==0)axis=probeAxis;
                    if(probeAxis.sqrMagnitude>0)
                    {
                        double projection=Vector3d.Dot(axis,probeAxis);
                        probeOmega*=projection;probeRateIntegral*=projection;probeTorqueIntegral*=projection;
                    }
                    probeAxis=axis;
                    double angle=Math.Acos(Math.Max(-1,Math.Min(1,Vector3d.Dot(probeForward,desiredDirection))));
                    double alpha=snapshot.TerminalAngularAcceleration*authorityScale;
                    double error=angle-probeOmega*Math.Abs(probeOmega)/(2*alpha);
                    double rate=ClampedPI(error,1,.1,alpha*snapshot.TerminalStoppingTime,step,ref probeRateIntegral);
                    double next=probeOmega+step*ClampedPI(rate-probeOmega,4,4,alpha,step,ref probeTorqueIntegral);
                    double turn=.5*(probeOmega+next)*step;probeOmega=next;
                    if(axis.sqrMagnitude>0)probeForward=(probeForward*Math.Cos(turn)+
                        Vector3d.Cross(axis,probeForward)*Math.Sin(turn)).normalized;
                    endForward=probeForward;
                }
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
                        previous.UT + fraction * (state.UT - previous.UT),
                        priorDeltaV + fraction * (deltaV - priorDeltaV));
                    deltaV = priorDeltaV + fraction * (deltaV - priorDeltaV);
                }
                trajectory.Add(state);
                if (reached) break;
            }
            double endSpeed = (state.Velocity - Vector3d.Cross(snapshot.AngularVelocity,
                state.Position)).magnitude;
            return new AirlessTargetAwareOutput(brakeReferenceUT, coast, trajectory,
                reached, endSpeed, deltaV, steps, true, mass, endForward, appliedThrottle,
                false, firstRequiredThrust,steeringReserveMaintained);
        }

        private static double V1AllowedSpeed(AirlessTargetAwareSnapshot snapshot,
            Vector3d position, double mass)
        {
            double availableThrust = snapshot.PolicyThrust * snapshot.InitialMass / mass;
            return V1LandingControlPolicy.SafeDescentMaximumSpeed(position.magnitude,
                snapshot.PolicyTerrainRadius, snapshot.PolicyGravity, availableThrust);
        }

        internal static AirlessTargetAwareState RK4WithThrust(AirlessTargetAwareState state,
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
            snapshot = AtDeliveryState(snapshot);
            ValidateSnapshot(snapshot);
            var samples = new List<AirlessTargetAwareState>();
            AirlessTargetAwareState state = new AirlessTargetAwareState(snapshot.Position,
                snapshot.Velocity, snapshot.StateUT);
            double period = 2 * Math.PI * Math.Sqrt(Math.Pow(snapshot.Position.magnitude, 3) / snapshot.BodyMu);
            double horizon = snapshot.StateUT + period;
            double minimumRadius = snapshot.BodyRadius + snapshot.MinimumTerrainASL;
            bool seenDescending = false;
            int coastSteps = 0;
            double lastStoredUT = double.NegativeInfinity;
            while (state.UT < horizon && samples.Count < 4096)
            {
                if (++coastSteps > MaximumCoastSteps)
                    throw new InvalidOperationException("Ballistic coast step limit exceeded");
                double radialMotion = Vector3d.Dot(state.Position, state.Velocity);
                if (radialMotion < 0) seenDescending = true;
                double altitude = state.Position.magnitude - snapshot.BodyRadius;
                // Retain a sparse, terrain-free orbital corridor outside the
                // PQS envelope so braking candidates can be searched before an
                // unpowered orbit physically intersects terrain. Keep the
                // existing one-second spacing where terrain can constrain the
                // path. The planner skips PQS for the sparse high-altitude
                // states, so this adds no main-thread terrain queries.
                double spacing = altitude <= snapshot.MaximumTerrainASL ? 1 : 10;
                if (state.UT - lastStoredUT >= spacing ||
                    altitude <= snapshot.MaximumTerrainASL)
                {
                    samples.Add(state);
                    lastStoredUT = state.UT;
                }
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
                !Finite(snapshot.StateUT) || !Finite(snapshot.VerticalPidPreviousError) ||
                !Finite(snapshot.VerticalPidIntegral) || snapshot.TerminalPhase < 0 || snapshot.TerminalPhase > 2 ||
                !Finite(snapshot.TerminalControllerBottomOffset) || snapshot.TerminalControllerBottomOffset < 0 ||
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
