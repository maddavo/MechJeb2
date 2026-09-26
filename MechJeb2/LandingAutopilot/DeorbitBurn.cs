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

            private bool _deorbitBurnTriggered;
            private double _deorbitThrottle;
            private double _targetNormalAngle;
            private double _targetAheadAngle;
            private double _planeChangeAngle;

            public DeorbitBurn(MechJebCore core) : base(core) { }

            public override string TraceDetails =>
                $" targetNormalAngle={_targetNormalAngle:F2} targetAheadAngle={_targetAheadAngle:F2} " +
                $"planeChangeAngle={_planeChangeAngle:F2} deorbitTriggered={_deorbitBurnTriggered}";

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
                double timeToImpact = Orbit.PerturbedOrbit(VesselState.Time, periapsisChange)
                    .NextTimeOfRadius(VesselState.Time, MainBody.Radius) - VesselState.Time;
                double rotationDegrees = 360.0 * timeToImpact / MainBody.rotationPeriod;
                Vector3d targetRadial = MainBody.GetWorldSurfacePosition(
                    Core.Target.targetLatitude, Core.Target.targetLongitude, 0.0) - MainBody.position;
                Vector3d futureRadial = Quaternion.AngleAxis((float)rotationDegrees, MainBody.angularVelocity) * targetRadial;
                Vector3d futureTarget = MainBody.position + futureRadial;
                Vector3d horizontalToTarget = Vector3d.Exclude(VesselState.Up, futureTarget - VesselState.CoM).normalized;
                Vector3d horizontalVelocity = Vector3d.Exclude(VesselState.Up, VesselState.OrbitalVelocity);
                Vector3d finalVelocity = horizontalVelocity + periapsisChange;
                Vector3d aimedVelocity = finalVelocity.magnitude * horizontalToTarget;
                Vector3d currentRadial = VesselState.CoM - MainBody.position;
                _targetNormalAngle = Vector3d.Angle(Orbit.OrbitNormal(), futureRadial);
                _targetNormalAngle = Math.Min(_targetNormalAngle, 180.0 - _targetNormalAngle);
                _targetAheadAngle = Vector3d.Angle(currentRadial, futureRadial);
                _planeChangeAngle = Vector3d.Angle(horizontalVelocity, horizontalToTarget);

                // Being in the target plane is necessary, but it does not identify a
                // deorbit opportunity. The former normal-angle shortcut could begin a
                // burn anywhere in the orbit and was observed to start roughly 43
                // degrees early. Require the same target-phase corridor for every
                // deorbit burn so the initial endpoint is acquired near the target.
                if (DeorbitBurnStartPolicy.IsInTargetingCorridor(_targetNormalAngle,
                        _targetAheadAngle, _planeChangeAngle))
                    _deorbitBurnTriggered = true;

                if (_deorbitBurnTriggered)
                {
                    if (!MuUtils.PhysicsRunning()) Core.Warp.MinimumWarp();

                    Vector3d burn = aimedVelocity - horizontalVelocity;
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
        }

        public static class DeorbitBurnStartPolicy
        {
            public static bool IsInTargetingCorridor(double targetNormalAngle, double targetAheadAngle,
                double planeChangeAngle)
            {
                return IsFinite(targetNormalAngle) && IsFinite(targetAheadAngle) && IsFinite(planeChangeAngle) &&
                    targetNormalAngle < 10.0 && targetAheadAngle > 60.0 && targetAheadAngle < 90.0 &&
                    planeChangeAngle < 90.0;
            }

            private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
        }
    }
}
