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
            private const int TargetingIterations = 5;
            private const double CandidatePlanningInterval = 1.0;

            private bool _deorbitBurnTriggered;
            private double _deorbitThrottle;
            private double _targetNormalAngle;
            private double _targetAheadAngle;
            private double _planeChangeAngle;
            private double _candidateImpactUT;
            private double _candidateEndpointError;
            private bool _candidateHasImpact;
            private double _lastCandidatePlanUT = double.NaN;
            private Vector3d _candidateFutureRadial;

            public DeorbitBurn(MechJebCore core) : base(core) { }

            public override string TraceDetails =>
                $" targetNormalAngle={_targetNormalAngle:F2} targetAheadAngle={_targetAheadAngle:F2} " +
                $"planeChangeAngle={_planeChangeAngle:F2} candidateImpactUT={_candidateImpactUT:F2} " +
                $"candidateEndpointError={_candidateEndpointError:F1} candidateHasImpact={_candidateHasImpact} " +
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
                if (!_deorbitBurnTriggered &&
                    (!DeorbitBurnStartPolicy.IsFinite(_lastCandidatePlanUT) ||
                     VesselState.Time - _lastCandidatePlanUT >= CandidatePlanningInterval))
                {
                    _candidateHasImpact = TrySolveTargetedBurn(periapsisChange, targetRadial, horizontalVelocity,
                        out _, out _candidateFutureRadial);
                    _lastCandidatePlanUT = VesselState.Time;
                }

                Vector3d futureRadial = _candidateFutureRadial;
                Vector3d burn = BuildTargetedBurn(periapsisChange, horizontalVelocity, futureRadial);
                Vector3d currentRadial = VesselState.CoM - MainBody.position;
                _targetNormalAngle = Vector3d.Angle(Orbit.OrbitNormal(), futureRadial);
                _targetNormalAngle = Math.Min(_targetNormalAngle, 180.0 - _targetNormalAngle);
                _targetAheadAngle = Vector3d.Angle(currentRadial, futureRadial);
                _planeChangeAngle = Vector3d.Angle(horizontalVelocity, horizontalVelocity + burn);

                // The target-normal value describes the target's position around
                // the orbital plane; it is near 90 degrees for an equatorial
                // target in an equatorial orbit. It is diagnostic only, not a
                // plane-alignment test. The old normal-angle shortcut could begin
                // a burn anywhere in the orbit and was observed to start roughly
                // 43 degrees early. The candidate burn and the rotating target
                // are solved together. A phase corridor alone is insufficient:
                // it previously committed a burn whose first predicted endpoint
                // missed the target by 185 km.
                if (DeorbitBurnStartPolicy.IsInTargetingCorridor(_targetNormalAngle,
                        _targetAheadAngle, _planeChangeAngle) &&
                    DeorbitBurnStartPolicy.IsCandidateAcceptable(_candidateHasImpact, _candidateEndpointError,
                        CandidateEndpointCorridor))
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

            private double CandidateEndpointCorridor => Math.Max(1000.0, MainBody.Radius * 0.01);

            /// <summary>
            /// The original controller rotated the target using the impact time of a pure
            /// periapsis-lowering burn, then executed a different redirected burn. This
            /// bounded fixed-point solve uses the impact time of the actual candidate burn.
            /// </summary>
            private bool TrySolveTargetedBurn(Vector3d periapsisChange, Vector3d targetRadial,
                Vector3d horizontalVelocity, out Vector3d burn, out Vector3d futureRadial)
            {
                burn = periapsisChange;
                futureRadial = targetRadial;
                _candidateImpactUT = double.NaN;
                _candidateEndpointError = double.NaN;

                Orbit initialOrbit = Orbit.PerturbedOrbit(VesselState.Time, periapsisChange);
                double impactUT = initialOrbit.NextTimeOfRadius(VesselState.Time, MainBody.Radius);
                if (!DeorbitBurnStartPolicy.IsFinite(impactUT) || impactUT <= VesselState.Time)
                    return false;

                for (int iteration = 0; iteration < TargetingIterations; iteration++)
                {
                    double rotationDegrees = 360.0 * (impactUT - VesselState.Time) / MainBody.rotationPeriod;
                    futureRadial = Quaternion.AngleAxis((float)rotationDegrees, MainBody.angularVelocity) * targetRadial;
                    Vector3d futureTarget = MainBody.position + futureRadial;
                    Vector3d horizontalToTarget =
                        Vector3d.Exclude(VesselState.Up, futureTarget - VesselState.CoM).normalized;
                    if (!DeorbitBurnStartPolicy.IsFiniteVector(horizontalToTarget))
                        return false;

                    burn = BuildTargetedBurn(periapsisChange, horizontalVelocity, futureRadial);
                    Orbit candidateOrbit = Orbit.PerturbedOrbit(VesselState.Time, burn);
                    double candidateImpactUT = candidateOrbit.NextTimeOfRadius(VesselState.Time, MainBody.Radius);
                    if (!DeorbitBurnStartPolicy.IsFinite(candidateImpactUT) || candidateImpactUT <= VesselState.Time)
                        return false;

                    impactUT = candidateImpactUT;
                }

                double finalRotationDegrees = 360.0 * (impactUT - VesselState.Time) / MainBody.rotationPeriod;
                futureRadial = Quaternion.AngleAxis((float)finalRotationDegrees, MainBody.angularVelocity) * targetRadial;
                Vector3d finalFutureTarget = MainBody.position + futureRadial;
                Vector3d finalHorizontalToTarget =
                    Vector3d.Exclude(VesselState.Up, finalFutureTarget - VesselState.CoM).normalized;
                if (!DeorbitBurnStartPolicy.IsFiniteVector(finalHorizontalToTarget))
                    return false;

                burn = BuildTargetedBurn(periapsisChange, horizontalVelocity, futureRadial);
                Orbit finalOrbit = Orbit.PerturbedOrbit(VesselState.Time, burn);
                impactUT = finalOrbit.NextTimeOfRadius(VesselState.Time, MainBody.Radius);
                if (!DeorbitBurnStartPolicy.IsFinite(impactUT) || impactUT <= VesselState.Time)
                    return false;

                double finalImpactRotationDegrees = 360.0 * (impactUT - VesselState.Time) / MainBody.rotationPeriod;
                futureRadial = Quaternion.AngleAxis((float)finalImpactRotationDegrees, MainBody.angularVelocity) * targetRadial;
                Vector3d endpoint = finalOrbit.WorldBCIPositionAtUT(impactUT) - MainBody.position;
                _candidateImpactUT = impactUT;
                _candidateEndpointError = Vector3d.Distance(endpoint, futureRadial);
                return DeorbitBurnStartPolicy.IsFiniteVector(endpoint) &&
                    DeorbitBurnStartPolicy.IsFinite(_candidateEndpointError);
            }

            private Vector3d BuildTargetedBurn(Vector3d periapsisChange, Vector3d horizontalVelocity,
                Vector3d futureRadial)
            {
                Vector3d futureTarget = MainBody.position + futureRadial;
                Vector3d horizontalToTarget =
                    Vector3d.Exclude(VesselState.Up, futureTarget - VesselState.CoM).normalized;
                return (horizontalVelocity + periapsisChange).magnitude * horizontalToTarget - horizontalVelocity;
            }
        }

        public static class DeorbitBurnStartPolicy
        {
            public static bool IsInTargetingCorridor(double targetNormalAngle, double targetAheadAngle,
                double planeChangeAngle)
            {
                return IsFinite(targetNormalAngle) && IsFinite(targetAheadAngle) && IsFinite(planeChangeAngle) &&
                    targetAheadAngle > 60.0 && targetAheadAngle < 90.0 &&
                    planeChangeAngle < 90.0;
            }

            public static bool IsCandidateAcceptable(bool hasImpact, double endpointError, double corridor)
            {
                return hasImpact && IsFinite(endpointError) && IsFinite(corridor) && corridor > 0 &&
                    endpointError <= corridor;
            }

            public static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

            public static bool IsFiniteVector(Vector3d value) =>
                IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }
    }
}
