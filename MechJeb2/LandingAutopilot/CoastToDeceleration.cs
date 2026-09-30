using System;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class CoastToDeceleration : AutopilotStep
        {
            private const double RcsEnableCorrectionDv = 1.5;
            private const double RcsDisableCorrectionDv = 0.5;
            private const double MaximumRcsCorrectionDv = 3.0;
            private const double MaximumRcsCommandChange = 0.5;

            private bool _haveRcsCommand;
            private long _lastRcsPredictionVersion = -1;
            private Vector3d _rcsCorrectionCommand;

            public CoastToDeceleration(MechJebCore core) : base(core)
            {
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (!Core.Landing.PredictionReady)
                    return this;
                if (!Core.Landing.RCSAdjustment)
                    return this;

                if (Core.Landing.PredictionVersion != _lastRcsPredictionVersion)
                {
                    _lastRcsPredictionVersion = Core.Landing.PredictionVersion;
                    Vector3d correction = LimitMagnitude(Core.Landing.ComputeCourseCorrection(true), MaximumRcsCorrectionDv);
                    _rcsCorrectionCommand = _haveRcsCommand
                        ? MoveTowards(_rcsCorrectionCommand, correction, MaximumRcsCommandChange)
                        : correction;
                    _haveRcsCommand = true;
                }

                if (!_haveRcsCommand)
                    return this;

                if (_rcsCorrectionCommand.magnitude > RcsEnableCorrectionDv)
                    Core.RCS.Enabled = true;
                else if (_rcsCorrectionCommand.magnitude < RcsDisableCorrectionDv)
                    Core.RCS.Enabled = false;

                if (Core.RCS.Enabled)
                    Core.RCS.SetWorldVelocityError(_rcsCorrectionCommand);

                return this;
            }

            private static Vector3d LimitMagnitude(Vector3d vector, double maximumMagnitude)
            {
                double magnitude = vector.magnitude;
                return magnitude > maximumMagnitude ? vector * (maximumMagnitude / magnitude) : vector;
            }

            private static Vector3d MoveTowards(Vector3d current, Vector3d target, double maximumChange)
            {
                Vector3d difference = target - current;
                double distance = difference.magnitude;
                return distance > maximumChange ? current + difference * (maximumChange / distance) : target;
            }

            private bool _warpReady;

            public override AutopilotStep OnFixedUpdate()
            {
                Core.Thrust.TargetThrottle = 0;

                // If the atmospheric drag is has started to act on the vessel then we are in a position to start considering when to deploy the parachutes.
                if (Core.Landing.DeployChutes)
                {
                    if (Core.Landing.ParachutesDeployable())
                    {
                        Core.Landing.ControlParachutes();
                    }
                }

                double maxAllowedSpeed = Core.Landing.MaxAllowedSpeed();
                if (VesselState.SpeedSurface > 0.9 * maxAllowedSpeed)
                {
                    Core.Warp.MinimumWarp();
                    if (Core.Landing.RCSAdjustment)
                        Core.RCS.Enabled = false;
                    return new DecelerationBurn(Core);
                }

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status1"); //"Coasting toward deceleration burn"

                if (Core.Landing.LandAtTarget)
                {
                    double currentError = Vector3d.Distance(Core.Target.GetPositionTargetPosition(), Core.Landing.LandingSite);
                    if (currentError > 1000)
                    {
                        if (!VesselState.ParachuteDeployed &&
                            VesselState.DragAcceleration <=
                            0.1) // However if there is already a parachute deployed or drag is high, then do not bother trying to correct the course as we will not have any attitude control anyway.
                        {
                            Core.Warp.MinimumWarp();
                            if (Core.Landing.RCSAdjustment)
                                Core.RCS.Enabled = false;
                            return new CourseCorrection(Core);
                        }
                    }
                    else
                    {
                        Vector3d deltaV = Core.Landing.ComputeCourseCorrection(true);
                        Status += "\n" + Localizer.Format("#MechJeb_LandingGuidance_Status2",
                            deltaV.magnitude.ToString("F3")); //"Course correction DV: " +  + " m/s"
                    }
                }

                // If we're already low, skip directly to the Deceleration burn
                if (VesselState.AltitudeASL < Core.Landing.DecelerationEndAltitude() + 5)
                {
                    Core.Warp.MinimumWarp();
                    if (Core.Landing.RCSAdjustment)
                        Core.RCS.Enabled = false;
                    return new DecelerationBurn(Core);
                }

                if (Core.Attitude.attitudeAngleFromTarget() < 1) { _warpReady = true; } // less warp start warp stop jumping

                if (Core.Attitude.attitudeAngleFromTarget() > 5) { _warpReady = false; } // hopefully

                if (Core.Landing.PredictionReady)
                {
                    if (VesselState.DragAcceleration < 0.01)
                    {
                        double decelerationStartTime = Core.Landing.Prediction.Trajectory.Any()
                            ? Core.Landing.Prediction.Trajectory.First().UT
                            : VesselState.Time;
                        Vector3d decelerationStartAttitude = -Orbit.WorldOrbitalVelocityAtUT(decelerationStartTime);
                        decelerationStartAttitude += MainBody.getRFrmVel(Orbit.WorldPositionAtUT(decelerationStartTime));
                        decelerationStartAttitude = decelerationStartAttitude.normalized;
                        Core.Attitude.attitudeTo(decelerationStartAttitude, AttitudeReference.INERTIAL, Core.Landing);
                    }
                    else
                    {
                        Core.Attitude.attitudeTo(Vector3.back, AttitudeReference.SURFACE_VELOCITY, Core.Landing);
                    }
                }

                //Warp at a rate no higher than the rate that would have us impacting the ground 10 seconds from now:
                if (_warpReady && Core.Node.Autowarp)
                {
                    // Make sure if we're hovering that we don't go straight into too fast of a warp
                    // (g * 5 is average velocity falling for 10 seconds from a hover)
                    double velocityGuess = Math.Max(Math.Abs(VesselState.SpeedVertical), VesselState.LocalGravity * 5);
                    Core.Warp.WarpRegularAtRate((float)(VesselState.AltitudeASL / (10 * velocityGuess)));
                }
                else
                {
                    Core.Warp.MinimumWarp();
                }

                return this;
            }
        }
    }
}
