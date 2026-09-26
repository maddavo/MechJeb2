using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        // Solver Basis V1: restored from the verified 7320... DLL. This is the
        // geometric controller, not the later experimental deorbit planner.
        public class DeorbitBurn : AutopilotStep
        {
            private const double MinimumDeorbitAcceleration = 3.0;
            private const double MaximumDeorbitTwr = 4.0;
            private const double DeorbitBurnTimeConstant = 0.5;
            private const double MinimumTerminalDeltaV = 0.05;
            // Each evaluation constructs several osculating orbits. Ten seconds of
            // vessel time is much smaller than the 30 degree ignition window, while
            // avoiding a simulation burst on every warp physics frame.
            private const double CandidatePlanningInterval = 10.0;
            private const double CandidateSearchExtentDegrees = 90.0;
            private const double CandidateSearchStepDegrees = 15.0;
            private const int CandidateRefinementIterations = 5;

            private bool _deorbitBurnTriggered;
            private double _deorbitThrottle;
            private double _lastCandidatePlanUT = double.NaN;
            private DeorbitCandidate _candidate;

            public DeorbitBurn(MechJebCore core) : base(core) { }

            public override string TraceDetails =>
                $" candidateValid={_candidate.Valid} candidateEndpointError={_candidate.EndpointError:F1} " +
                $"candidateAimRotation={_candidate.AimRotationDegrees:F2} " +
                $"deorbitTriggered={_deorbitBurnTriggered}";

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 5.0)
                    Core.Thrust.RequestActiveThrottle((float)_deorbitThrottle, enforceMinimum: true, allowZero: true);
                else if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 10.0 && Core.Thrust.LimiterMinThrottle)
                    Core.Thrust.RequestActiveThrottle(0f, enforceMinimum: true, allowZero: true);
                else
                    Core.Thrust.ThrustOff();

                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                if (Orbit.ApA < MainBody.RealMaxAtmosphereAltitude())
                {
                    Core.Thrust.ThrustOff();
                    return new CourseCorrection(Core);
                }

                Vector3d periapsisChange = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(
                    Orbit, VesselState.Time, 0.9 * MainBody.Radius);
                Vector3d targetRadial = MainBody.GetWorldSurfacePosition(
                    Core.Target.targetLatitude, Core.Target.targetLongitude, 0.0) - MainBody.position;
                Vector3d horizontalVelocity = Vector3d.Exclude(VesselState.Up, VesselState.OrbitalVelocity);
                if (!_deorbitBurnTriggered && (double.IsNaN(_lastCandidatePlanUT) ||
                    VesselState.Time - _lastCandidatePlanUT >= CandidatePlanningInterval))
                {
                    _candidate = FindCandidate(periapsisChange, targetRadial, horizontalVelocity);
                    _lastCandidatePlanUT = VesselState.Time;
                }

                Vector3d futureRadial = _candidate.AimRadial;
                Vector3d burn = _candidate.Burn;
                Vector3d currentRadial = VesselState.CoM - MainBody.position;
                double targetNormalAngle = Vector3d.Angle(Orbit.OrbitNormal(), futureRadial);
                targetNormalAngle = Math.Min(targetNormalAngle, 180.0 - targetNormalAngle);
                double targetAheadAngle = Vector3d.Angle(currentRadial, futureRadial);
                double planeChangeAngle = Vector3d.Angle(horizontalVelocity, horizontalVelocity + burn);

                if (_candidate.Valid && _candidate.EndpointError <= Math.Max(1000.0, MainBody.Radius * 0.01) &&
                    targetAheadAngle < 90.0 && targetAheadAngle > 60.0 && planeChangeAngle < 90.0)
                    _deorbitBurnTriggered = true;

                if (_deorbitBurnTriggered)
                {
                    if (!MuUtils.PhysicsRunning()) Core.Warp.MinimumWarp();

                    Core.Attitude.attitudeTo(burn.normalized, AttitudeReference.INERTIAL, Core.Landing);
                    double maxThrustAcceleration = VesselState.LimitedMaxThrustAcceleration;
                    if (maxThrustAcceleration <= 0.0)
                    {
                        Core.Thrust.ThrustOff();
                        return this;
                    }

                    double bodyAwareMaximum = Math.Max(MinimumDeorbitAcceleration, MaximumDeorbitTwr * MainBody.GeeASL * 9.81);
                    double cappedAcceleration = Math.Min(maxThrustAcceleration, bodyAwareMaximum);
                    double responseDeltaV = VesselState.CurrentThrustAcceleration * VesselState.MaxEngineResponseTime;
                    double desiredAcceleration = Math.Max(0.0, (burn.magnitude - responseDeltaV) / DeorbitBurnTimeConstant);
                    _deorbitThrottle = Math.Min(desiredAcceleration / maxThrustAcceleration, cappedAcceleration / maxThrustAcceleration);
                    double terminalDeltaV = Math.Max(MinimumTerminalDeltaV,
                        cappedAcceleration * (TimeWarp.fixedDeltaTime + VesselState.MaxEngineResponseTime));
                    if (burn.magnitude <= terminalDeltaV)
                    {
                        Core.Thrust.ThrustOff();
                        return new CourseCorrection(Core);
                    }

                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status7");
                }
                else
                {
                    Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.ORBIT, Core.Landing);
                    if (Core.Node.Autowarp) Core.Warp.WarpRegularAtRate((float)(Orbit.period / 10.0));
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status8");
                }

                return this;
            }

            private DeorbitCandidate FindCandidate(Vector3d periapsisChange, Vector3d targetRadial,
                Vector3d horizontalVelocity)
            {
                try
                {
                    Orbit nominalOrbit = Orbit.PerturbedOrbit(VesselState.Time, periapsisChange);
                    double nominalImpactUT = nominalOrbit.NextTimeOfRadius(VesselState.Time, MainBody.Radius);
                    if (!IsFinite(nominalImpactUT) || nominalImpactUT <= VesselState.Time)
                        return default;

                    double nominalRotation = 360.0 * (nominalImpactUT - VesselState.Time) / MainBody.rotationPeriod;
                    DeorbitCandidate best = default;
                    for (double offset = -CandidateSearchExtentDegrees; offset <= CandidateSearchExtentDegrees;
                         offset += CandidateSearchStepDegrees)
                        ConsiderCandidate(ref best, EvaluateCandidate(nominalRotation + offset, periapsisChange,
                            targetRadial, horizontalVelocity));
                    RefineCandidate(ref best, periapsisChange, targetRadial, horizontalVelocity);
                    return best;
                }
                catch (ArgumentException)
                {
                    return default;
                }
            }

            private void RefineCandidate(ref DeorbitCandidate best, Vector3d periapsisChange, Vector3d targetRadial,
                Vector3d horizontalVelocity)
            {
                if (!best.Valid) return;
                double low = best.AimRotationDegrees - CandidateSearchStepDegrees;
                double high = best.AimRotationDegrees + CandidateSearchStepDegrees;
                for (int iteration = 0; iteration < CandidateRefinementIterations; iteration++)
                {
                    double left = (2 * low + high) / 3;
                    double right = (low + 2 * high) / 3;
                    DeorbitCandidate leftCandidate = EvaluateCandidate(left, periapsisChange, targetRadial, horizontalVelocity);
                    DeorbitCandidate rightCandidate = EvaluateCandidate(right, periapsisChange, targetRadial, horizontalVelocity);
                    ConsiderCandidate(ref best, leftCandidate);
                    ConsiderCandidate(ref best, rightCandidate);
                    if (!leftCandidate.Valid || rightCandidate.Valid && leftCandidate.EndpointError > rightCandidate.EndpointError)
                        low = left;
                    else
                        high = right;
                }
            }

            private DeorbitCandidate EvaluateCandidate(double aimRotationDegrees, Vector3d periapsisChange,
                Vector3d targetRadial, Vector3d horizontalVelocity)
            {
                try
                {
                    Vector3d aimRadial = Quaternion.AngleAxis((float)aimRotationDegrees, MainBody.angularVelocity) * targetRadial;
                    Vector3d futureTarget = MainBody.position + aimRadial;
                    Vector3d horizontalToTarget = Vector3d.Exclude(VesselState.Up, futureTarget - VesselState.CoM).normalized;
                    if (!IsFiniteVector(horizontalToTarget)) return default;
                    Vector3d finalVelocity = horizontalVelocity + periapsisChange;
                    Vector3d burn = finalVelocity.magnitude * horizontalToTarget - horizontalVelocity;
                    Orbit candidateOrbit = Orbit.PerturbedOrbit(VesselState.Time, burn);
                    double impactUT = candidateOrbit.NextTimeOfRadius(VesselState.Time, MainBody.Radius);
                    if (!IsFinite(impactUT) || impactUT <= VesselState.Time) return default;
                    // WorldBCIPositionAtUT is already body-centred. Subtracting
                    // MainBody.position here shifts the endpoint by a second body
                    // origin and makes an otherwise valid burn look hundreds of
                    // kilometres off target.
                    Vector3d endpoint = candidateOrbit.WorldBCIPositionAtUT(impactUT);
                    double targetRotation = 360.0 * (impactUT - VesselState.Time) / MainBody.rotationPeriod;
                    Vector3d actualTarget = Quaternion.AngleAxis((float)targetRotation, MainBody.angularVelocity) * targetRadial;
                    if (!IsFiniteVector(endpoint) || !IsFiniteVector(actualTarget)) return default;
                    return new DeorbitCandidate { Valid = true, AimRotationDegrees = aimRotationDegrees, AimRadial = aimRadial,
                        Burn = burn, EndpointError = Vector3d.Distance(endpoint, actualTarget) };
                }
                catch (ArgumentException)
                {
                    return default;
                }
            }

            private static void ConsiderCandidate(ref DeorbitCandidate best, DeorbitCandidate candidate)
            {
                if (candidate.Valid && (!best.Valid || candidate.EndpointError < best.EndpointError)) best = candidate;
            }

            private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
            private static bool IsFiniteVector(Vector3d value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);

            private struct DeorbitCandidate
            {
                public bool Valid;
                public double AimRotationDegrees;
                public Vector3d AimRadial;
                public Vector3d Burn;
                public double EndpointError;
            }
        }
    }
}
