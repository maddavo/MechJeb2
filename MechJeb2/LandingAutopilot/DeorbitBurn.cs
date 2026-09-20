using System;
using KSP.Localization;
using UnityEngine;

// FIXME: use a maneuver node

namespace MuMech
{
    namespace Landing
    {
        public class DeorbitBurn : AutopilotStep
        {
            // Keep high-TWR vessels controllable while preserving a useful burn rate
            // on higher-gravity bodies.
            private const double MinimumDeorbitAcceleration = 3.0;
            private const double MaximumDeorbitTwr = 4.0;
            private const double DeorbitBurnTimeConstant = 0.5;
            private const double MinimumTerminalDeltaV = 0.05;
            private const int PeriapsisPlanSamples = 20;
            private const int HeadingPlanIterations = 4;

            private bool _deorbitBurnTriggered;
            private bool _hasDeorbitPlan;
            private bool _burnCommanded;
            private double _deorbitThrottle;
            private double _plannedDeltaV;
            private double _plannedBurnDv;
            private double _deliveredBurnDv;
            private Vector3d _plannedBurnDirection;
            private double _plannedLongError;

            public DeorbitBurn(MechJebCore core) : base(core)
            {
            }

            public override string TraceDetails =>
                $" plannedDv={_plannedDeltaV:F3} plannedLongError={_plannedLongError:F1} " +
                $"throttle={_deorbitThrottle:F3}";

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 5)
                {
                    Core.Thrust.RequestActiveThrottle((float)_deorbitThrottle, allowZero: true);
                    _burnCommanded = true;
                }
                else if (_deorbitBurnTriggered && Core.Attitude.attitudeAngleFromTarget() < 10 && Core.Thrust.LimiterMinThrottle)
                {
                    Core.Thrust.RequestActiveThrottle(0, allowZero: true);
                    _burnCommanded = false;
                }
                else
                {
                    Core.Thrust.ThrustOff();
                    _burnCommanded = false;
                }

                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                // If we are already on a re-entry trajectory, high-deorbit geometry
                // cannot improve the entry. Let the normal correction stage take over.
                if (Orbit.ApA < MainBody.RealMaxAtmosphereAltitude())
                {
                    Core.Thrust.ThrustOff();
                    return new CourseCorrection(Core);
                }

                if (!_deorbitBurnTriggered)
                {
                    if (!ShouldStartDeorbit())
                    {
                        Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.ORBIT, Core.Landing);
                        if (Core.Node.Autowarp) Core.Warp.WarpRegularAtRate((float)(Orbit.period / 10));
                        Status = Localizer.Format("#MechJeb_LandingGuidance_Status8");
                        return this;
                    }

                    _deorbitBurnTriggered = true;
                }

                if (!MuUtils.PhysicsRunning()) Core.Warp.MinimumWarp();

                double maximumAcceleration = VesselState.LimitedMaxThrustAcceleration;
                if (maximumAcceleration <= 0)
                {
                    Core.Thrust.ThrustOff();
                    return this;
                }

                // Build one inertial, node-like burn plan.  Unlike the previous
                // percentage-reserve experiment, this searches the orbital geometry for
                // a body-rotation-aware impact deliberately beyond the target.  The plan
                // is immutable once firing starts; prediction may observe it, but cannot
                // cause the deorbit stage to chase a moving marker.
                double bodyAwareAccelerationCap = Math.Max(MinimumDeorbitAcceleration,
                    MaximumDeorbitTwr * MainBody.GeeASL * 9.81);
                double limitedAcceleration = Math.Min(maximumAcceleration, bodyAwareAccelerationCap);
                if (!_hasDeorbitPlan)
                {
                    if (!TryCreateDeorbitPlan(out _plannedBurnDirection, out _plannedBurnDv, out _plannedLongError))
                    {
                        Core.Thrust.ThrustOff();
                        Core.Landing.TraceLanding("deorbit plan unavailable; handing off without an unbounded burn");
                        return new CourseCorrection(Core);
                    }

                    _hasDeorbitPlan = true;
                    Core.Landing.TraceLanding($"deorbit plan dv={_plannedBurnDv:F3} " +
                        $"longError={_plannedLongError:F1} targetLong={LongSideCorridorCenter:F1}");
                }

                if (_burnCommanded)
                    _deliveredBurnDv += VesselState.CurrentThrustAcceleration * TimeWarp.fixedDeltaTime;

                _plannedDeltaV = Math.Max(0, _plannedBurnDv - _deliveredBurnDv);
                double responseDeltaV = VesselState.CurrentThrustAcceleration * VesselState.MaxEngineResponseTime;
                if (_plannedDeltaV <= responseDeltaV + MinimumTerminalDeltaV)
                {
                    Core.Thrust.ThrustOff();
                    _burnCommanded = false;
                    Core.Landing.TraceLanding($"deorbit handoff reason=planned-long-corridor " +
                        $"deliveredDv={_deliveredBurnDv:F3} remainingPlannedDv={_plannedDeltaV:F3}");
                    return new CourseCorrection(Core);
                }

                Core.Attitude.attitudeTo(_plannedBurnDirection, AttitudeReference.INERTIAL, Core.Landing);

                double desiredAcceleration = Math.Max(0, (_plannedDeltaV - responseDeltaV) / DeorbitBurnTimeConstant);
                _deorbitThrottle = Math.Min(desiredAcceleration / maximumAcceleration,
                    limitedAcceleration / maximumAcceleration);

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status7");
                return this;
            }

            private bool ShouldStartDeorbit()
            {
                Vector3d targetAtImpact = TargetAtGeometricImpact();
                Vector3d currentRadialVector = VesselState.CoM - MainBody.position;
                Vector3d currentHorizontalVelocity = Vector3d.Exclude(VesselState.Up, VesselState.OrbitalVelocity);
                Vector3d horizontalToTarget = Vector3d.Exclude(VesselState.Up, targetAtImpact - VesselState.CoM).normalized;
                double targetAngleToOrbitNormal = Vector3d.Angle(Orbit.OrbitNormal(), targetAtImpact - MainBody.position);
                targetAngleToOrbitNormal = Math.Min(targetAngleToOrbitNormal, 180 - targetAngleToOrbitNormal);
                double targetAheadAngle = Vector3d.Angle(currentRadialVector, targetAtImpact - MainBody.position);
                double planeChangeAngle = Vector3d.Angle(currentHorizontalVelocity, horizontalToTarget);

                return targetAngleToOrbitNormal < 10 ||
                    (targetAheadAngle < 90 && targetAheadAngle > 60 && planeChangeAngle < 90);
            }

            private Vector3d GeometricDeorbitDeltaV()
            {
                Vector3d horizontalDV = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(Orbit, VesselState.Time,
                    0.9 * MainBody.Radius);
                Vector3d currentHorizontalVelocity = Vector3d.Exclude(VesselState.Up, VesselState.OrbitalVelocity);
                double finalHorizontalSpeed = (currentHorizontalVelocity + horizontalDV).magnitude;
                Vector3d horizontalToTarget = Vector3d.Exclude(VesselState.Up, TargetAtGeometricImpact() - VesselState.CoM).normalized;
                return finalHorizontalSpeed * horizontalToTarget - currentHorizontalVelocity;
            }

            private double LongSideCorridorCenter
            {
                get
                {
                    double acceleration = Math.Max(0.1, VesselState.LimitedMaxThrustAcceleration);
                    double brakingDistance = VesselState.SpeedSurface * VesselState.SpeedSurface / (2 * acceleration);
                    double near = Math.Max(100, MainBody.Radius * 0.0005);
                    double far = Math.Max(2 * near, Math.Min(MainBody.Radius * 0.01, brakingDistance * 0.5));
                    return 0.5 * (near + far);
                }
            }

            private bool TryCreateDeorbitPlan(out Vector3d direction, out double deltaV, out double longError)
            {
                direction = Vector3d.zero;
                deltaV = 0;
                longError = double.NaN;
                double desiredLongError = LongSideCorridorCenter;
                double bestScore = double.PositiveInfinity;

                // A shallow periapsis produces a smaller, longer trajectory; a deep one
                // produces a larger, shorter trajectory.  Search that physically monotonic
                // family instead of taking a full-depth burn and subtracting an arbitrary
                // fraction afterwards.
                for (int sample = 0; sample <= PeriapsisPlanSamples; sample++)
                {
                    double fraction = 0.90 + 0.099 * sample / PeriapsisPlanSamples;
                    if (!TryBuildPlanForPeriapsis(fraction * MainBody.Radius, desiredLongError,
                            out Vector3d candidate, out double candidateLongError))
                        continue;

                    double score = Math.Abs(candidateLongError - desiredLongError);
                    if (score < bestScore)
                    {
                        bestScore = score;
                        direction = candidate.normalized;
                        deltaV = candidate.magnitude;
                        longError = candidateLongError;
                    }
                }

                // A plan outside the requested corridor is not an approximate success:
                // firing it would recreate the short-side overshoot we are preventing.
                double maximumPlanError = Math.Max(100, 0.5 * desiredLongError);
                return deltaV > MinimumTerminalDeltaV && bestScore <= maximumPlanError;
            }

            private bool TryBuildPlanForPeriapsis(double periapsisRadius, double desiredLongError,
                out Vector3d deltaV, out double longError)
            {
                deltaV = Vector3d.zero;
                longError = double.NaN;
                Vector3d horizontalDV = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(Orbit, VesselState.Time, periapsisRadius);
                Vector3d currentHorizontalVelocity = Vector3d.Exclude(VesselState.Up, VesselState.OrbitalVelocity);
                double finalHorizontalSpeed = (currentHorizontalVelocity + horizontalDV).magnitude;
                deltaV = GeometricDeorbitDeltaV();

                for (int iteration = 0; iteration < HeadingPlanIterations; iteration++)
                {
                    Orbit candidateOrbit = Orbit.PerturbedOrbit(VesselState.Time, deltaV);
                    if (candidateOrbit.PeR >= MainBody.Radius)
                        return false;

                    double impactUT = candidateOrbit.NextTimeOfRadius(VesselState.Time, MainBody.Radius);
                    if (double.IsNaN(impactUT) || double.IsInfinity(impactUT))
                        return false;

                    Vector3d targetRadial = TargetRadialAtUT(impactUT);
                    Vector3d groundTrack = Vector3d.Exclude(targetRadial,
                        candidateOrbit.WorldOrbitalVelocityAtUT(impactUT)).normalized;
                    Vector3d longSideAim = (targetRadial + desiredLongError * groundTrack).normalized * MainBody.Radius;
                    Vector3d horizontalToAim = Vector3d.Exclude(VesselState.Up,
                        MainBody.position + longSideAim - VesselState.CoM).normalized;
                    deltaV = finalHorizontalSpeed * horizontalToAim - currentHorizontalVelocity;
                }

                Orbit finalOrbit = Orbit.PerturbedOrbit(VesselState.Time, deltaV);
                if (finalOrbit.PeR >= MainBody.Radius)
                    return false;
                double finalImpactUT = finalOrbit.NextTimeOfRadius(VesselState.Time, MainBody.Radius);
                Vector3d finalTarget = TargetRadialAtUT(finalImpactUT);
                // WorldBCIPositionAtUT is already body-centred.  Subtracting the body
                // position again mixes coordinate frames and makes an otherwise valid
                // candidate appear tens of kilometres from its actual impact point.
                Vector3d finalImpact = finalOrbit.WorldBCIPositionAtUT(finalImpactUT);
                Vector3d finalGroundTrack = Vector3d.Exclude(finalTarget,
                    finalOrbit.WorldOrbitalVelocityAtUT(finalImpactUT)).normalized;
                longError = Vector3d.Dot(Vector3d.Exclude(finalTarget, finalImpact - finalTarget), finalGroundTrack);
                return !double.IsNaN(longError) && Vector3d.Dot(deltaV, currentHorizontalVelocity) < 0;
            }

            private Vector3d TargetRadialAtUT(double ut)
            {
                Vector3d target = MainBody.GetWorldSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, 0) - MainBody.position;
                double rotation = 360 * (ut - VesselState.Time) / MainBody.rotationPeriod;
                return Quaternion.AngleAxis((float)rotation, MainBody.angularVelocity) * target;
            }

            private Vector3d TargetAtGeometricImpact()
            {
                Vector3d horizontalDV = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(Orbit, VesselState.Time,
                    0.9 * MainBody.Radius);
                Orbit forwardDeorbitTrajectory = Orbit.PerturbedOrbit(VesselState.Time, horizontalDV);
                double freefallTime = forwardDeorbitTrajectory.NextTimeOfRadius(VesselState.Time, MainBody.Radius) - VesselState.Time;
                Vector3d currentTargetRadialVector =
                    MainBody.GetWorldSurfacePosition(Core.Target.targetLatitude, Core.Target.targetLongitude, 0) - MainBody.position;
                var rotation = Quaternion.AngleAxis((float)(360 * freefallTime / MainBody.rotationPeriod), MainBody.angularVelocity);
                return MainBody.position + rotation * currentTargetRadialVector;
            }
        }
    }
}
