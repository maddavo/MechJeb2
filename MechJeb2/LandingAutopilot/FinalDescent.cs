using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class FinalDescent : AutopilotStep
        {
            // Terminal descent needs a speed envelope as well as a braking-distance
            // calculation.  The latter alone produces impractically large allowed
            // speeds for high-TWR vessels on low-gravity bodies.
            private const double MinimumTerminalDescentSpeed = 5.0;
            private const double MaximumTerminalDescentSpeed = 25.0;
            private const double TerminalDescentGravityTime = 4.0;
            private IDescentSpeedPolicy _aggressivePolicy;

            public FinalDescent(MechJebCore core) : base(core)
            {
            }

            public override AutopilotStep OnFixedUpdate()
            {
                return this;
                /*double minalt = Math.Min(VesselState.altitudeBottom, Math.Min(VesselState.altitudeASL, VesselState.altitudeTrue));

                if (!Core.Node.Autowarp || _aggressivePolicy == null) return this;

                double maxVel = 1.02 * _aggressivePolicy.MaxAllowedSpeed(VesselState.CoM - MainBody.position, VesselState.surfaceVelocity);

                double diffPercent = (maxVel / VesselState.speedSurface - 1) * 100;

                if (minalt > 200 && diffPercent > 0 && Vector3d.Angle(VesselState.forward, -VesselState.surfaceVelocity) < 45)
                    Core.Warp.WarpRegularAtRate((float)(diffPercent * diffPercent * diffPercent));
                else
                    Core.Warp.MinimumWarp(true);

                return this;*/
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (Vessel.LandedOrSplashed)
                {
                    Core.Landing.StopLanding();
                    return null;
                }

                // TODO perhaps we should pop the parachutes at this point, or at least consider it depending on the altitude.

                double minalt = Math.Min(VesselState.AltitudeBottom, Math.Min(VesselState.AltitudeASL, VesselState.AltitudeTrue));

                if (VesselState.LimitedMaxThrustAcceleration < VesselState.GravityForce.magnitude)
                {
                    // if we have TWR < 1, just try as hard as we can to decelerate:
                    // (we need this special case because otherwise the calculations spit out NaN's)
                    Core.Thrust.Tmode = MechJebModuleThrustController.TMode.KEEP_VERTICAL;
                    Core.Thrust.TransKillH = true;
                    Core.Thrust.TransSpdAct = 0;
                }
                else if (minalt > 300)
                {
                    if (VesselState.SurfaceVelocity.magnitude > 5 && Vector3d.Angle(VesselState.SurfaceVelocity, VesselState.Up) < 80)
                    {
                        // if we have positive vertical velocity, point up and follow min thrust limiter:
                        Core.Attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, null);
                        Core.Thrust.Tmode = MechJebModuleThrustController.TMode.DIRECT;
                        Core.Thrust.TransSpdAct = Core.Thrust.LimiterMinThrottle ? 100 * (float)Core.Thrust.MinThrottle : 0;
                    }
                    else if (VesselState.SurfaceVelocity.magnitude > 5 && Vector3d.Angle(VesselState.Forward, -VesselState.SurfaceVelocity) > 45)
                    {
                        // if we're not facing approximately retrograde, turn to point retrograde and follow min thrust limiter:
                        Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, null);
                        Core.Thrust.Tmode = MechJebModuleThrustController.TMode.DIRECT;
                        Core.Thrust.TransSpdAct = Core.Thrust.LimiterMinThrottle ? 100 * (float)Core.Thrust.MinThrottle : 0;
                    }
                    else
                    {
                        //if we're above 300m, point retrograde and control surface velocity:
                        Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, null);

                        Core.Thrust.Tmode = MechJebModuleThrustController.TMode.KEEP_SURFACE;

                        //core.thrust.trans_spd_act = (float)Math.Sqrt((vesselState.maxThrustAccel - vesselState.gravityForce.magnitude) * 2 * minalt) * 0.90F;
                        Vector3d estimatedLandingPosition = VesselState.CoM + VesselState.SurfaceVelocity.sqrMagnitude /
                            (2 * VesselState.LimitedMaxThrustAcceleration) * VesselState.SurfaceVelocity.normalized;
                        double terrainRadius = MainBody.Radius + MainBody.TerrainAltitude(estimatedLandingPosition);
                        _aggressivePolicy =
                            new GravityTurnDescentSpeedPolicy(terrainRadius, MainBody.GeeASL * 9.81,
                                VesselState.LimitedMaxThrustAcceleration); // this constant policy creation is wastefull...
                        Core.Thrust.TransSpdAct =
                            (float)_aggressivePolicy.MaxAllowedSpeed(VesselState.CoM - MainBody.position, VesselState.SurfaceVelocity);
                    }
                }
                else
                {
                    // last 300 meters:
                    double netBrakingAcceleration = Math.Max(0,
                        VesselState.LimitedMaxThrustAcceleration - VesselState.LocalGravity);
                    double brakingDistanceSpeed = Math.Sqrt(2 * netBrakingAcceleration * Math.Max(0, minalt)) * 0.90;
                    double bodyAwareSpeedCap = Math.Max(MinimumTerminalDescentSpeed,
                        Math.Min(MaximumTerminalDescentSpeed, TerminalDescentGravityTime * VesselState.LocalGravity));
                    double desiredSpeed = -Math.Min(brakingDistanceSpeed, bodyAwareSpeedCap);

                    // At this height a direct, full-throttle retrograde burn can turn a
                    // few metres per second of drift into an ascent on a high-TWR craft.
                    // Keep vertical speed under closed-loop control and let the thrust
                    // controller remove the remaining lateral velocity at the same time.
                    Core.Thrust.Tmode = MechJebModuleThrustController.TMode.KEEP_VERTICAL;
                    Core.Thrust.TransSpdAct = (float)Math.Min(-Core.Landing.TouchdownSpeed, desiredSpeed);
                    Core.Thrust.TransKillH = true;
                }

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status9",
                    VesselState.AltitudeBottom.ToString("F0")); //"Final descent: " +  + "m above terrain"

                // ComputeCourseCorrection doesn't work close to the ground
                /* if (core.landing.landAtTarget)
                {
                    core.rcs.enabled = true;
                    Vector3d deltaV = core.landing.ComputeCourseCorrection(false);
                    core.rcs.SetWorldVelocityError(deltaV);

                    status += "\nDV: " + deltaV.magnitude.ToString("F3") + " m/s";
                } */

                return this;
            }
        }
    }
}
