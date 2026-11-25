using System;
using System.Threading;

namespace MuMech.Landing
{
    // A correction-guidance experiment. Its forecasts can never be published
    // as the current vessel forecast. Uses the existing V1 braking policy.
    internal sealed class CorrectionBurnPlan
    {
        internal readonly AirlessTargetAwareSnapshot Initial, AfterBurn;
        internal readonly AirlessTargetAwareOutput Forecast;
        internal readonly Vector3d Direction;
        internal readonly double BurnStartUT, Delivery, BaselineError, ExpectedError;
        internal CorrectionBurnPlan(AirlessTargetAwareSnapshot initial, AirlessTargetAwareSnapshot after,
            AirlessTargetAwareOutput forecast, Vector3d direction, double start, double delivery,
            double baselineError, double expectedError)
        {
            Initial = initial; AfterBurn = after; Forecast = forecast; Direction = direction;
            BurnStartUT = start; Delivery = delivery; BaselineError = baselineError; ExpectedError = expectedError;
        }
    }

    internal static class CorrectionBurnPlanner
    {
        internal const int MaximumTrials = 8;
        internal const int MaximumBurnSteps = 20000;

        internal static bool CapabilityCompatible(double current, double nominal, double flow, double nominalFlow) =>
            nominal > 0 && !double.IsNaN(current) && !double.IsInfinity(current) &&
            Math.Abs(current - nominal) <= Math.Max(1e-9, nominal * 1e-4) &&
            Math.Abs(flow - nominalFlow) <= Math.Max(1e-12, nominalFlow * 1e-4);

        internal static AirlessTargetAwareSnapshot PhysicalState(AirlessTargetAwareSnapshot s,
            AirlessTargetAwareState state, double mass, Vector3d forward)
        {
            var raw = new AirlessTargetAwareSnapshot(s.InputUT, s.EpochUT, s.BodyRadius, s.BodyMu,
                s.BodyGeeASL, s.RotationPeriod, s.TargetLatitude, s.TargetLongitude, s.TargetTerrainASL,
                s.DecelEndASL, s.MaximumThrustAcceleration * s.InitialMass / mass, s.Dt, s.MinDt,
                s.MaxOrbits, s.MinimumTerrainASL, s.MaximumTerrainASL, state.Position, state.Velocity,
                s.AngularVelocity, s.Axis0, s.Axis90, s.AxisNorth, stateUT: state.UT);
            return new AirlessTargetAwareSnapshot(raw, mass, s.MaximumThrust, s.MinimumThrust,
                s.MaximumMassFlow, s.MinimumMassFlow, s.PolicyTerrainRadius, s.PolicyGravity,
                s.PolicyThrust * s.InitialMass / mass, false, forward, s.MinimumCommandThrottle,
                s.MaximumCommandThrottle, s.ThrottleSmoothingSeconds, 0, s.TouchdownSpeed, s.BottomOffset,
                s.PreviousTransThrottle, s.SafeTouchdownSpeed, s.TerminalBottomOffset,
                s.TerminalControllerBottomOffset, 0, 0, 0, false, s.TerminalAngularAcceleration,
                s.TerminalAngularSpeed, s.TerminalStoppingTime);
        }

        internal static AirlessTargetAwareSnapshot? Coast(AirlessTargetAwareSnapshot s, double until,
            Func<bool> cancelled)
        {
            var state = new AirlessTargetAwareState(s.Position, s.Velocity, s.StateUT);
            if (until < state.UT || until - state.UT > 120) return null;
            while (state.UT < until)
            {
                if (cancelled()) throw new OperationCanceledException();
                state = AirlessTargetAwareSimulation.RK4WithThrust(state,
                    Math.Min(.2, until - state.UT), s.BodyMu, Vector3d.zero);
                if (state.Position.magnitude <= s.BodyRadius + s.MaximumTerrainASL + s.BottomOffset) return null;
            }
            return PhysicalState(s, state, s.InitialMass, s.InitialForward);
        }

        internal static AirlessTargetAwareSnapshot? Burn(AirlessTargetAwareSnapshot s, Vector3d direction,
            double delivery, double deadline, Func<bool> cancelled)
        {
            if (!(delivery > 0) || s.MinimumCommandThrottle != 0 || s.MinimumThrust != 0 ||
                s.ThrottleSmoothingSeconds != 0 || direction.sqrMagnitude == 0) return null;
            var state = new AirlessTargetAwareState(s.Position, s.Velocity, s.StateUT);
            double remaining = V1LandingControlPolicy.CorrectionCommandForDelivery(delivery), mass = s.InitialMass;
            direction = direction.normalized;
            for (int i = 0; i < MaximumBurnSteps; i++)
            {
                if (cancelled()) throw new OperationCanceledException();
                if (remaining <= .05) return PhysicalState(s, state, mass, direction);
                if (state.UT + s.MinDt >= deadline || state.Position.magnitude <=
                    s.BodyRadius + s.MaximumTerrainASL + s.BottomOffset) return null;
                // Exact existing instantaneous-engine ThrustForDv command.
                // Engines with lag/smoothing use the established live fallback.
                double maximum = s.MaximumThrust / mass;
                double throttle = Math.Min(s.MaximumCommandThrottle,
                    Math.Max(.01, Math.Min(1, remaining / (.5 * maximum))));
                double acceleration = throttle * maximum;
                state = AirlessTargetAwareSimulation.RK4WithThrust(state, s.MinDt, s.BodyMu, direction * acceleration);
                mass -= throttle * s.MaximumMassFlow * s.MinDt;
                remaining -= acceleration * s.MinDt;
                if (!(mass > s.InitialMass * .01)) return null;
            }
            return null;
        }

        internal static double Error(AirlessTargetAwareSnapshot s, AirlessTargetAwareState end)
        {
            Vector3d target = AirlessTargetAwareSimulation.SurfaceDirection(s.TargetLatitude, s.TargetLongitude, s);
            return (BrakingEndpointResponse.BodyPoint(s, end).normalized - target).magnitude * s.BodyRadius;
        }

        internal static CorrectionBurnPlan Search(AirlessTargetAwareSnapshot s, Vector3d direction,
            double requested, double conservativeDelivery, double start, double reference, double deadline,
            Func<bool> cancelled)
        {
            var coast = Coast(s, start, cancelled);
            if (coast == null || !(requested > conservativeDelivery * 2) || reference - 5 <= start) return null;
            var baseline = AirlessTargetAwareSimulation.RunCoastForecast(coast.Value, reference);
            if (!baseline.ReachedHandoff || baseline.AttitudeUncertain) return null;
            var terminal = AirlessTargetAwareSimulation.RunNominalTerminal(coast.Value, baseline, s.DecelEndASL - 200);
            if (terminal?.FinalDescent == null || !terminal.FinalDescent.ReachedTerrain) return null;
            double before = Error(s, terminal.End);
            CorrectionBurnPlan best = null;
            // Nonlinear finite-burn backtracking, not a looser tangent bound.
            for (int i = 0; i < MaximumTrials; i++)
            {
                if (cancelled()) throw new OperationCanceledException();
                double dv = requested * .5 / Math.Pow(2, i);
                if (dv <= conservativeDelivery * 1.5) break;
                var after = Burn(coast.Value, direction, dv, Math.Min(deadline, reference - 5), cancelled);
                if (after == null) continue;
                var forecast = AirlessTargetAwareSimulation.RunCoastForecast(after.Value, reference);
                if (!forecast.ReachedHandoff || forecast.AttitudeUncertain ||
                    !TargetAwareTerminalHandoff.Assess(after.Value, forecast, s.DecelEndASL - 200).NecessaryControlBoundPasses) continue;
                var end = AirlessTargetAwareSimulation.RunNominalTerminal(after.Value, forecast, s.DecelEndASL - 200);
                if (end?.FinalDescent == null || !end.FinalDescent.ReachedTerrain) continue;
                double error = Error(s, end.End);
                if (error >= before || double.IsNaN(error) || double.IsInfinity(error)) continue;
                if (best == null || error < best.ExpectedError)
                    best = new CorrectionBurnPlan(coast.Value, after.Value, forecast, direction.normalized, start, dv, before, error);
            }
            return best;
        }
    }
}
