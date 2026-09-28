using System;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class DecelerationBurn : AutopilotStep
        {
            private bool _decelerationBurnTriggered;

            internal bool BrakingTriggered => _decelerationBurnTriggered;

            public DecelerationBurn(MechJebCore core) : base(core)
            {
            }

            public override AutopilotStep OnFixedUpdate()
            {
                if (V1LandingControlPolicy.BelowTerminalHandoff(VesselState.AltitudeASL,
                        Core.Landing.DecelerationEndAltitude()))
                {
                    Core.Warp.MinimumWarp();

                    if (Core.Landing.UseAtmosphereToBrake())
                        return new FinalDescent(Core);
                    return new KillHorizontalVelocity(Core);
                }

                double decelerationStartTime = Core.Landing.Prediction.BrakeReferenceUT(VesselState.Time);
                if (V1LandingControlPolicy.ShouldCoastToBrake(decelerationStartTime,
                        VesselState.Time, _decelerationBurnTriggered))
                {
                    Core.Thrust.ThrustOff();

                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status4"); //"Warping to start of braking burn."

                    //warp to deceleration start
                    Vector3d decelerationStartAttitude = -Orbit.WorldOrbitalVelocityAtUT(decelerationStartTime);
                    decelerationStartAttitude += MainBody.getRFrmVel(Orbit.WorldPositionAtUT(decelerationStartTime));
                    decelerationStartAttitude = decelerationStartAttitude.normalized;
                    Core.Attitude.attitudeTo(decelerationStartAttitude, AttitudeReference.INERTIAL, Core.Landing);
                    bool warpReady = V1LandingControlPolicy.BrakeWarpReady(
                        Core.Attitude.attitudeAngleFromTarget(), Core.vessel.angularVelocity.magnitude);

                    if (warpReady && Core.Node.Autowarp)
                        Core.Warp.WarpToUT(decelerationStartTime - 5);

                    else if (!MuUtils.PhysicsRunning())
                        Core.Warp.MinimumWarp();
                    return this;
                }

                if (!_decelerationBurnTriggered)
                    _decelerationBurnTriggered = true;

                Vector3d courseCorrection = Core.Landing.ComputeCourseCorrection(false);
                Vector3d desiredThrustVector = V1LandingControlPolicy.BrakeThrustDirection(
                    VesselState.SurfaceVelocity, courseCorrection, VesselState.LimitedMaxThrustAcceleration);

                if (V1LandingControlPolicy.BrakeAttitudeGate(
                        Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up),
                        Vector3d.Dot(VesselState.Forward, desiredThrustVector)))
                {
                    Core.Thrust.RequestActiveThrottle(0.0f);
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status5"); //"Braking"
                }
                else
                {
                    double controlledSpeed =
                        VesselState.SpeedSurface *
                        Math.Sign(Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up)); //positive if we are ascending, negative if descending
                    double desiredSpeed = -Core.Landing.MaxAllowedSpeed();
                    double desiredSpeedAfterDt = -Core.Landing.MaxAllowedSpeedAfterDt(VesselState.DeltaT);
                    double minAccel = -VesselState.LocalGravity * Math.Abs(Vector3d.Dot(VesselState.SurfaceVelocity.normalized, VesselState.Up));
                    double maxAccel = VesselState.MaxThrustAcceleration * Vector3d.Dot(VesselState.Forward, -VesselState.SurfaceVelocity.normalized) -
                        VesselState.LocalGravity * Math.Abs(Vector3d.Dot(VesselState.SurfaceVelocity.normalized, VesselState.Up));
                    Core.Thrust.RequestActiveThrottle(V1LandingControlPolicy.BrakingThrottle(
                        controlledSpeed, desiredSpeed, desiredSpeedAfterDt,
                        VesselState.DeltaT, minAccel, maxAccel));
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status6",
                        desiredSpeed >= double.MaxValue ? "∞" : Math.Abs(desiredSpeed).ToString("F1")); //"Braking: target speed = " +  + " m/s"
                }

                Core.Attitude.attitudeTo(desiredThrustVector, AttitudeReference.INERTIAL, Core.Landing);

                return this;
            }
        }
    }
}
