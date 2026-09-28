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
            private bool _deorbitBurnTriggered;
            private double _deorbitThrottle;

            public DeorbitBurn(MechJebCore core) : base(core) { }

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
                double targetNormalAngle = Vector3d.Angle(Orbit.OrbitNormal(), futureRadial);
                targetNormalAngle = Math.Min(targetNormalAngle, 180.0 - targetNormalAngle);
                double targetAheadAngle = Vector3d.Angle(currentRadial, futureRadial);
                double planeChangeAngle = Vector3d.Angle(horizontalVelocity, horizontalToTarget);

                if (V1LandingControlPolicy.ShouldTriggerGeometricDeorbit(
                        targetNormalAngle, targetAheadAngle, planeChangeAngle))
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

                    _deorbitThrottle = V1LandingControlPolicy.GeometricDeorbitThrottle(
                        burn.magnitude, maxThrustAcceleration,
                        VesselState.CurrentThrustAcceleration,
                        VesselState.MaxEngineResponseTime, MainBody.GeeASL);
                    if (V1LandingControlPolicy.GeometricDeorbitComplete(
                            burn.magnitude, maxThrustAcceleration, MainBody.GeeASL,
                            TimeWarp.fixedDeltaTime, VesselState.MaxEngineResponseTime))
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
    }
}
