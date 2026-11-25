using System;
using System.Linq;
using KSP.Localization;
using UnityEngine;
using System.Globalization;

namespace MuMech
{
    namespace Landing
    {
        public class DecelerationBurn : AutopilotStep
        {
            private bool _decelerationBurnTriggered;
            private bool _dunaBrakingCommanded, _dunaForceBraking;
            private bool _poweredBrakingObserved, _coastWarpOwned;
            private double _firstActualThrustUT = double.NaN, _warpExitUT = double.NaN;
            private string _warpReason = "reference_wait";
            private bool _poweredResponseUsed;
            private double _correctionMagnitude,_steeringRatio,_correctionHorizon;
            private Vector3d _correctionVector,_thrustCommand;
            public override string TraceDetails => string.Format(CultureInfo.InvariantCulture,
                " poweredBrakingObserved={0} firstActualThrustUT={1:R} warpExitUT={2:R} warpReason={3} measuredTurnSeconds={4:R} measuredSettleSeconds={5:R} actualWarpExitUT={6:R} residualAngularSpeed={7:R}",
                _poweredBrakingObserved, _firstActualThrustUT, _warpExitUT, _warpReason,
                Core.Landing.WarpPreparation.TurnSeconds, Core.Landing.WarpPreparation.SettleSeconds,
                Core.Landing.WarpPreparation.ActualWarpExitUT, Core.Landing.WarpPreparation.ResidualAngularSpeed) +
                string.Format(CultureInfo.InvariantCulture,
                    " poweredResponseUsed={0} brakingCorrectionMagnitude={1:R} brakingSteeringRatio={2:R} brakingCorrectionHorizon={3:R}",
                    _poweredResponseUsed,_correctionMagnitude,_steeringRatio,_correctionHorizon) +
                string.Format(CultureInfo.InvariantCulture,
                    " brakingCorrection=({0:R},{1:R},{2:R}) brakingThrustCommand=({3:R},{4:R},{5:R})",
                    _correctionVector.x,_correctionVector.y,_correctionVector.z,
                    _thrustCommand.x,_thrustCommand.y,_thrustCommand.z) +
                (Core.Landing.DunaBrakingCommitted ? " dunaForceBraking="+_dunaForceBraking : string.Empty);

            private void CancelCoastWarp()
            {
                if (!_coastWarpOwned) return;
                if (Core.Landing.WarpPreparation.OwnsWarpTarget(Core.Warp.warpToUT))
                    Core.Warp.MinimumWarp();
                Core.Landing.WarpPreparation.ClearWarpRequest();
                _coastWarpOwned = false;
            }

            // Read-only predictor snapshot flag; does not change V1 decisions.
            internal bool BrakingTriggered => _decelerationBurnTriggered;

            public DecelerationBurn(MechJebCore core) : base(core)
            {
            }

            public override AutopilotStep OnFixedUpdate()
            {
                if (Core.Landing.DunaDynamicActive && !Core.Landing.DunaBrakingCommitted)
                {
                    if (!Core.Landing.CheckDunaBrakingHandoff())
                    {
                        Core.Thrust.TargetThrottle=0;
                        return new CoastToDeceleration(Core);
                    }
                }
                bool dunaCommitted=Core.Landing.DunaBrakingCommitted;
                if (_decelerationBurnTriggered && VesselState.CurrentThrustAcceleration > 0 &&
                    (!dunaCommitted || _dunaBrakingCommanded &&
                        Vector3d.Dot(VesselState.Forward,-VesselState.SurfaceVelocity.normalized)>0.75))
                {
                    if (!_poweredBrakingObserved) _firstActualThrustUT = VesselState.Time;
                    _poweredBrakingObserved = true;
                }
                if (dunaCommitted && Core.Landing.DunaDynamicTerminalReady)
                {
                    Core.Warp.MinimumWarp();
                    Core.Thrust.RequestActiveThrottle(0);
                    return new KillHorizontalVelocity(Core);
                }
                if (!dunaCommitted && VesselState.AltitudeASL < Core.Landing.DecelerationEndAltitude() + 5)
                {
                    Core.Warp.MinimumWarp();

                    if (Core.Landing.UseAtmosphereToBrake())
                        return new FinalDescent(Core);
                    return new KillHorizontalVelocity(Core);
                }

                double decelerationStartTime = dunaCommitted ? VesselState.Time :
                    Core.Landing.Prediction.BrakeReferenceUT(VesselState.Time);
                if (decelerationStartTime - VesselState.Time > 5 && !_decelerationBurnTriggered)
                {
                    Core.Thrust.ThrustOff();

                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status4"); //"Warping to start of braking burn."

                    //warp to deceleration start
                    Vector3d decelerationStartAttitude = -Orbit.WorldOrbitalVelocityAtUT(decelerationStartTime);
                    decelerationStartAttitude += MainBody.getRFrmVel(Orbit.WorldPositionAtUT(decelerationStartTime));
                    decelerationStartAttitude = decelerationStartAttitude.normalized;
                    Core.Attitude.attitudeTo(decelerationStartAttitude, AttitudeReference.INERTIAL, Core.Landing);
                    Core.Landing.WarpPreparation.Observe(VesselState.Forward, decelerationStartAttitude,
                        Core.vessel.angularVelocity.magnitude, VesselState.DeltaT, MuUtils.PhysicsRunning());
                    bool warpReady = Core.Landing.WarpPreparation.ReadyForWarp(
                        Vector3d.Angle(VesselState.Forward, decelerationStartAttitude), MuUtils.PhysicsRunning(),
                        Core.Landing.WarpPreparation.OwnsWarpTarget(Core.Warp.warpToUT));

                    if (warpReady && Core.Node.Autowarp)
                    {
                        _coastWarpOwned = true;
                        Core.Landing.WarpPreparation.RequestedWarpExit(decelerationStartTime - 5);
                        Core.Warp.WarpToUT(decelerationStartTime - 5);
                    }

                    else if (!MuUtils.PhysicsRunning())
                        Core.Warp.MinimumWarp();
                    return this;
                }

                if (!_decelerationBurnTriggered)
                    _decelerationBurnTriggered = true;

                Vector3d desiredThrustVector = -VesselState.SurfaceVelocity.normalized;

                // An expired forecast exposes timing only, not an endpoint.
                // Continue the nominal braking law without optional steering;
                // never dereference its deliberately absent landing frame.
                _dunaForceBraking=dunaCommitted && Core.Landing.DunaBrakingRequiresThrust(_poweredBrakingObserved);
                Vector3d courseCorrection=Vector3d.zero;
                _poweredResponseUsed=false;
                if (!_dunaForceBraking)
                {
                    _poweredResponseUsed=Core.Landing.TryPoweredBrakingCorrection(out courseCorrection,
                        out _correctionHorizon);
                    if (!_poweredResponseUsed) courseCorrection=Core.Landing.PredictionReady &&
                        (!dunaCommitted || Core.Landing.DunaBrakingForecastUsable) ?
                        Core.Landing.ComputeCourseCorrection(false) : Vector3d.zero;
                }
                else _correctionHorizon=0;
                if (dunaCommitted && !DunaBrakingHandoff.Finite(courseCorrection.magnitude))
                    courseCorrection=Vector3d.zero;
                double correctionAngle = _poweredResponseUsed ? courseCorrection.magnitude :
                    courseCorrection.magnitude / (2.0 * VesselState.LimitedMaxThrustAcceleration);
                if (dunaCommitted && !DunaBrakingHandoff.Finite(correctionAngle)) correctionAngle=0;
                correctionAngle = Math.Min(_poweredResponseUsed ?
                    Math.Max(.1,BrakingEndpointResponse.MarginRatio(Core.Thrust.LastThrottle,Core.Thrust.ThrottleLimit)) : .1,
                    correctionAngle);
                _correctionMagnitude=courseCorrection.magnitude;_steeringRatio=correctionAngle;
                desiredThrustVector = (desiredThrustVector + correctionAngle * courseCorrection.normalized).normalized;
                _correctionVector=courseCorrection;_thrustCommand=desiredThrustVector;


                if (Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up) > 0
                    || Vector3d.Dot(VesselState.Forward, desiredThrustVector) < 0.75)
                {
                    Core.Thrust.RequestActiveThrottle(0.0f);
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status5"); //"Braking"
                }
                else if (_dunaForceBraking)
                {
                    // Do not wait for a less conservative or invalid legacy
                    // envelope. Retain alignment gate and thrust limiters.
                    Core.Thrust.RequestActiveThrottle(1.0f);
                    Status = "Duna braking: dynamic capture";
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
                    const double SPEED_CORRECTION_TIME_CONSTANT = 0.3;
                    double speedError = desiredSpeed - controlledSpeed;
                    double desiredAccel = speedError / SPEED_CORRECTION_TIME_CONSTANT + (desiredSpeedAfterDt - desiredSpeed) / VesselState.DeltaT;
                    if (maxAccel - minAccel > 0)
                        Core.Thrust.RequestActiveThrottle(Mathf.Clamp((float)((desiredAccel - minAccel) / (maxAccel - minAccel)), 0.0f, 1.0f));
                    else Core.Thrust.RequestActiveThrottle(0);
                    Status = Localizer.Format("#MechJeb_LandingGuidance_Status6",
                        desiredSpeed >= double.MaxValue ? "∞" : Math.Abs(desiredSpeed).ToString("F1")); //"Braking: target speed = " +  + " m/s"
                }

                if (dunaCommitted) _dunaBrakingCommanded=Core.Thrust.TargetThrottle>0;
                Core.Attitude.attitudeTo(desiredThrustVector, AttitudeReference.INERTIAL, Core.Landing);

                Core.Landing.WarpPreparation.Observe(VesselState.Forward, desiredThrustVector,
                    Core.vessel.angularVelocity.magnitude, VesselState.DeltaT, MuUtils.PhysicsRunning());
                ReentrySimulation.Result forecast = Core.Landing.Prediction;
                _warpExitUT = double.NaN;
                if (forecast != null && forecast.HasFirstRequiredThrust)
                {
                    Vector3d futureDirection = -Orbit.WorldOrbitalVelocityAtUT(forecast.FirstRequiredThrustUT) +
                        MainBody.getRFrmVel(Orbit.WorldPositionAtUT(forecast.FirstRequiredThrustUT));
                    _warpExitUT = Core.Landing.WarpPreparation.ExitUT(forecast.FirstRequiredThrustUT,
                        VesselState.Time, Vector3d.Angle(desiredThrustVector, futureDirection),
                        double.NaN, Time.fixedDeltaTime);
                }
                bool attitudeHeld = Vector3d.Dot(VesselState.Forward, desiredThrustVector) < 0.75;
                bool coastWarp = V1WarpPreparation.UnpoweredWarpAllowed(Core.Node.Autowarp,
                    Core.Landing.AirlessTargetedForecast, Core.Landing.PredictionReady &&
                    forecast != null && forecast.HasFirstRequiredThrust &&
                    VesselState.LimitedMaxThrustAcceleration > VesselState.LocalGravity, _poweredBrakingObserved,
                    attitudeHeld, VesselState.SpeedVertical > 0, Core.Thrust.TargetThrottle,
                    Vessel.ctrlState.mainThrottle, VesselState.CurrentThrustAcceleration,
                    Core.Thrust.LimiterMinThrottle ? (double)Core.Thrust.MinThrottle : 0,
                    VesselState.ThrustMinimum, VesselState.Time, forecast?.FirstRequiredThrustUT ?? double.NaN,
                    forecast?.EndUT ?? double.NaN, _warpExitUT);
                if (coastWarp && Core.Landing.WarpPreparation.ReadyForWarp(
                        Vector3d.Angle(VesselState.Forward, desiredThrustVector), MuUtils.PhysicsRunning(),
                        Core.Landing.WarpPreparation.OwnsWarpTarget(Core.Warp.warpToUT)))
                {
                    _warpReason = "validated_zero_throttle_coast";
                    _coastWarpOwned = true;
                    Core.Landing.WarpPreparation.RequestedWarpExit(_warpExitUT);
                    Core.Warp.WarpToUT(_warpExitUT);
                }
                else
                {
                    _warpReason = "preparation_or_thrust_or_forecast_unavailable";
                    CancelCoastWarp();
                }


                return this;
            }
        }
    }
}
