using System;
using KSP.Localization;
using System.Globalization;
using UnityEngine;

// FIXME: use a maneuver node

namespace MuMech
{
    namespace Landing
    {
        public class LowDeorbitBurn : AutopilotStep
        {
            private bool _deorbitBurnTriggered;
            private double _lowDeorbitBurnMaxThrottle;

            private bool _lowDeorbitEndConditionSet;
            private bool _lowDeorbitEndOnLandingSiteNearer;

            private const double LOW_DEORBIT_BURN_TRIGGER_FACTOR = 2;

            private V1DeorbitPreparation _preparation = new V1DeorbitPreparation();
            private double _nextPreparationEstimateUT;
            private double _estimatedTriggerUT = double.NaN;
            private double _preparationTargetLatitude = double.NaN, _preparationTargetLongitude = double.NaN;
            private double _reserveSeconds = double.NaN, _warpExitUT = double.NaN;

            public override string TraceDetails => string.Format(CultureInfo.InvariantCulture,
                " deorbitTriggered={0} preparationStartedUT={1:R} prepared={2} preparationSeconds={3:R} estimatedTriggerUT={4:R} preparationLiveRetrograde={5} warpExitUT={6:R} orientationReserve={7:R} measuredTurnSeconds={8:R} measuredSettleSeconds={9:R} actualWarpExitUT={10:R}",
                _deorbitBurnTriggered, _preparation.StartedUT, _preparation.Complete,
                _preparation.Seconds, _estimatedTriggerUT, _preparation.LiveRetrograde,
                _warpExitUT, _reserveSeconds, Core.Landing.WarpPreparation.TurnSeconds,
                Core.Landing.WarpPreparation.SettleSeconds, Core.Landing.WarpPreparation.ActualWarpExitUT);

            private Vector3d PrepareDeorbitAttitude(double decelerationEndASL)
            {
                double now = VesselState.Time;
                // KSP inverse-rotates world coordinates and osculating LAN/
                // argument each tick. Those are not new manoeuvres. Refresh
                // geometry from the current orbit without resetting turn time.
                if (Core.Target.targetLatitude != _preparationTargetLatitude ||
                    Core.Target.targetLongitude != _preparationTargetLongitude)
                {
                    _preparation = new V1DeorbitPreparation();
                    _nextPreparationEstimateUT = 0;
                    _estimatedTriggerUT = double.NaN;
                    _reserveSeconds = double.NaN;
                    _preparationTargetLatitude = Core.Target.targetLatitude;
                    _preparationTargetLongitude = Core.Target.targetLongitude;
                }
                _preparation.Start(now);
                if (_preparation.UseLiveRetrograde(_estimatedTriggerUT, now, _reserveSeconds))
                    return -VesselState.SurfaceVelocity.normalized;
                if (now >= _nextPreparationEstimateUT)
                {
                    // This accessor already returns body-centred coordinates.
                    Vector3d target = Core.Target.GetPositionTargetPosition();
                    double acceleration = VesselState.LimitedMaxThrustAcceleration;
                    _estimatedTriggerUT = V1LandingControlPolicy.UpcomingDeorbitTrigger(ut =>
                    {
                        Vector3d position = Orbit.WorldBCIPositionAtUT(ut);
                        return V1LandingControlPolicy.DeorbitPreparationRangeError(position,
                            Orbit.WorldOrbitalVelocityAtUT(ut), target, MainBody.angularVelocity,
                            ut - now, MainBody.Radius, decelerationEndASL, acceleration);
                    }, now, Orbit.period);
                    _nextPreparationEstimateUT = now + 0.5;
                }
                if (double.IsNaN(_estimatedTriggerUT))
                    return -VesselState.SurfaceVelocity.normalized;
                Vector3d futurePosition = Orbit.WorldBCIPositionAtUT(_estimatedTriggerUT);
                Vector3d futureDirection = -(Orbit.WorldOrbitalVelocityAtUT(_estimatedTriggerUT) -
                    Vector3d.Cross(MainBody.angularVelocity, futurePosition)).normalized;
                _reserveSeconds = Core.Landing.WarpPreparation.Estimate(Vector3d.Angle(
                    futureDirection, -VesselState.SurfaceVelocity.normalized), _preparation.Seconds);
                if (_preparation.UseLiveRetrograde(_estimatedTriggerUT, now, _reserveSeconds))
                    return -VesselState.SurfaceVelocity.normalized;
                return futureDirection;
            }

            public LowDeorbitBurn(MechJebCore core) : base(core)
            {
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 5)
                {
                    Core.Thrust.RequestActiveThrottle(Mathf.Clamp01((float)_lowDeorbitBurnMaxThrottle), allowZero: true);
                }
                else if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 10 && Core.Thrust.LimiterMinThrottle)
                {
                    Core.Thrust.RequestActiveThrottle(0.0f);
                }
                else
                {
                    Core.Thrust.ThrustOff();
                }

                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                //Decide when we will start the deorbit burn:
                double stoppingDistance = Math.Pow(VesselState.SpeedSurfaceHorizontal, 2) / (2 * VesselState.LimitedMaxThrustAcceleration);
                double triggerDistance = LOW_DEORBIT_BURN_TRIGGER_FACTOR * stoppingDistance;
                double decelerationEndASL = Core.Landing.DecelerationEndAltitude();
                double heightAboveTarget = VesselState.AltitudeASL - decelerationEndASL;
                if (triggerDistance < heightAboveTarget)
                {
                    triggerDistance = heightAboveTarget;
                }

                //See if it's time to start the deorbit burn:
                double rangeToTarget = Vector3d.Exclude(VesselState.Up, Core.Target.GetPositionTargetPosition() - VesselState.CoM).magnitude;

                if (!_deorbitBurnTriggered && rangeToTarget < triggerDistance)
                {
                    if (!MuUtils.PhysicsRunning()) Core.Warp.MinimumWarp(true);
                    _deorbitBurnTriggered = true;
                }

                Status = Localizer.Format(_deorbitBurnTriggered
                    ? "#MechJeb_LandingGuidance_Status11"
                    : //"Executing low deorbit burn"
                    "#MechJeb_LandingGuidance_Status12"); //"Moving to low deorbit burn point"

                //By default, thrust straight back at max throttle
                Vector3d thrustDirection = !_deorbitBurnTriggered && Core.Node.Autowarp ?
                    PrepareDeorbitAttitude(decelerationEndASL) : -VesselState.SurfaceVelocity.normalized;
                _lowDeorbitBurnMaxThrottle = 1;

                //If we are burning, we watch the predicted landing site and switch to the braking
                //burn when the predicted landing site crosses the target. We also use the predictions
                //to steer the predicted landing site toward the target
                if (_deorbitBurnTriggered && Core.Landing.PredictionReady)
                {
                    //angle slightly left or right to fix any cross-range error in the predicted landing site:
                    Vector3d horizontalToLandingSite = Vector3d.Exclude(VesselState.Up, Core.Landing.LandingSite - VesselState.CoM).normalized;
                    Vector3d horizontalToTarget =
                        Vector3d.Exclude(VesselState.Up, Core.Target.GetPositionTargetPosition() - VesselState.CoM).normalized;
                    const double ANGLE_GAIN = 4;
                    Vector3d angleCorrection = ANGLE_GAIN * (horizontalToTarget - horizontalToLandingSite);
                    if (angleCorrection.magnitude > 0.1) angleCorrection *= 0.1 / angleCorrection.magnitude;
                    thrustDirection = (thrustDirection + angleCorrection).normalized;

                    double rangeToLandingSite = Vector3d.Exclude(VesselState.Up, Core.Landing.LandingSite - VesselState.CoM).magnitude;
                    double maxAllowedSpeed = Core.Landing.MaxAllowedSpeed();

                    if (!_lowDeorbitEndConditionSet &&
                        Vector3d.Distance(Core.Landing.LandingSite, VesselState.CoM) < MainBody.Radius + VesselState.AltitudeASL)
                    {
                        _lowDeorbitEndOnLandingSiteNearer = rangeToLandingSite > rangeToTarget;
                        _lowDeorbitEndConditionSet = true;
                    }

                    _lowDeorbitBurnMaxThrottle = 1;

                    if (Orbit.PeA < 0)
                    {
                        if (rangeToLandingSite > rangeToTarget)
                        {
                            if (_lowDeorbitEndConditionSet && !_lowDeorbitEndOnLandingSiteNearer)
                            {
                                Core.Thrust.ThrustOff();
                                return new DecelerationBurn(Core);
                            }

                            double maxAllowedSpeedAfterDt = Core.Landing.MaxAllowedSpeedAfterDt(VesselState.DeltaT);
                            double speedAfterDt = VesselState.SpeedSurface +
                                VesselState.DeltaT * Vector3d.Dot(VesselState.GravityForce, VesselState.SurfaceVelocity.normalized);
                            double throttleToMaintainLandingSite;
                            if (VesselState.SpeedSurface < maxAllowedSpeed) throttleToMaintainLandingSite = 0;
                            else
                                throttleToMaintainLandingSite =
                                    (speedAfterDt - maxAllowedSpeedAfterDt) / (VesselState.DeltaT * VesselState.MaxThrustAcceleration);

                            _lowDeorbitBurnMaxThrottle = throttleToMaintainLandingSite + 1 * (rangeToLandingSite / rangeToTarget - 1) + 0.2;
                        }
                        else
                        {
                            if (_lowDeorbitEndConditionSet && _lowDeorbitEndOnLandingSiteNearer)
                            {
                                Core.Thrust.ThrustOff();
                                return new DecelerationBurn(Core);
                            }

                            _lowDeorbitBurnMaxThrottle = 0.0f;
                            Status = Localizer.Format(
                                "#MechJeb_LandingGuidance_Status13"); //"Deorbit burn complete: waiting for the right moment to start braking"
                        }
                    }
                }

                Core.Attitude.attitudeTo(thrustDirection, AttitudeReference.INERTIAL, Core.Landing);

                if (!_deorbitBurnTriggered && Core.Node.Autowarp)
                {
                    Core.Landing.WarpPreparation.Observe(VesselState.Forward, thrustDirection,
                        Core.vessel.angularVelocity.magnitude, VesselState.DeltaT, MuUtils.PhysicsRunning());
                    // Test the newly issued direction, never the old plane-
                    // change command. Measure turn plus settling at physics rate.
                    bool ready = Core.Landing.WarpPreparation.ReadyForWarp(
                        Vector3d.Angle(VesselState.Forward, thrustDirection), MuUtils.PhysicsRunning(),
                        Core.Landing.WarpPreparation.OwnsWarpTarget(Core.Warp.warpToUT));
                    _preparation.Observe(ready, MuUtils.PhysicsRunning(), VesselState.Time, VesselState.DeltaT);
                    if (ready && _preparation.Complete && MuUtils.PhysicsRunning())
                        Core.Landing.WarpPreparation.RetainMeasuredPreparation(_preparation.Seconds);
                    _warpExitUT = _preparation.LiveRetrograde || !ready || !_preparation.Complete ? double.NaN :
                        Core.Landing.WarpPreparation.ExitUT(_estimatedTriggerUT, VesselState.Time,
                            Vector3d.Angle(thrustDirection, -VesselState.SurfaceVelocity.normalized),
                            _preparation.Seconds, Time.fixedDeltaTime);
                    if (!double.IsNaN(_warpExitUT))
                    {
                        Core.Landing.WarpPreparation.RequestedWarpExit(_warpExitUT);
                        Core.Warp.WarpToUT(_warpExitUT);
                    }
                    else if (Core.Landing.WarpPreparation.OwnsWarpTarget(Core.Warp.warpToUT))
                    {
                        Core.Warp.MinimumWarp();
                        Core.Landing.WarpPreparation.ClearWarpRequest();
                    }
                    else if (!MuUtils.PhysicsRunning()) Core.Warp.MinimumWarp();
                }
                else if (!_deorbitBurnTriggered && !Core.Node.Autowarp &&
                    Core.Landing.WarpPreparation.OwnsWarpTarget(Core.Warp.warpToUT))
                {
                    Core.Warp.MinimumWarp();
                    Core.Landing.WarpPreparation.ClearWarpRequest();
                }
                if (rangeToTarget < triggerDistance && !MuUtils.PhysicsRunning()) Core.Warp.MinimumWarp();

                return this;
            }
        }
    }
}
