using System;
using System.Runtime.Serialization;
using MechJebLib.Lambert;
using MechJebLib.Primitives;
using MuMech;
using UnityEngine;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class CapturedMunPlanningTests
    {
        [Fact]
        public void RecordedMunCoastMatchesTheLaterTraceSnapshot()
        {
            var trajectory = new AirlessConicTrajectory(6.5138398e10, 24108732.482233,
                new Vector3d(237305.616576, -4574.283068, -27494.511833),
                new Vector3d(60.255816, 5.298250, 518.597745));
            Assert.True(trajectory.TryStateAt(24108738.842233, out Vector3d position, out Vector3d velocity));
            Assert.True(Vector3d.Distance(position, new Vector3d(237658.944818, -4540.145469, -24262.014012)) < 100,
                "position=" + position);
            Assert.True(Vector3d.Distance(velocity, new Vector3d(53.192611, 5.436655, 519.367374)) < 0.2,
                "velocity=" + velocity);
        }

        [Fact]
        public void RecordedMunLambertTransferReachesTheRotatingTarget()
        {
            LandingGuidanceV2Snapshot snapshot = RecordedMunSnapshot();
            var coast = new AirlessConicTrajectory(snapshot.Body.gravParameter, snapshot.UT, snapshot.Position, snapshot.Velocity);
            double bestError = double.PositiveInfinity;
            double bestDeltaV = double.NaN;
            double bestCrossingDifference = double.NaN;
            double bestBurnUT = double.NaN;
            Vector3d bestPosition = Vector3d.zero;
            Vector3d bestVelocity = Vector3d.zero;
            for (int epochIndex = 0; epochIndex <= 48; ++epochIndex)
            {
                double burnUT = snapshot.UT + 2.0 + coast.Period * epochIndex / 48.0;
                Assert.True(coast.TryStateAt(burnUT, out Vector3d position, out Vector3d velocity));
                for (int flightIndex = 1; flightIndex <= 36; ++flightIndex)
                {
                    double flightTime = 45.0 + coast.Period * flightIndex / 36.0;
                    Vector3d target = RotateTarget(snapshot, burnUT + flightTime);
                    try
                    {
                        V3 positionV3 = ToV3(position);
                        V3 velocityV3 = ToV3(velocity);
                        (V3 transferVelocity, _) = Gooding.Solve(snapshot.Body.gravParameter, positionV3, ToV3(target),
                            flightTime, TransferGeometry.Prograde, 0, V3.Cross(positionV3, velocityV3));
                        var transfer = new AirlessConicTrajectory(snapshot.Body.gravParameter, burnUT, position, ToVector3d(transferVelocity));
                        if (!transfer.TryStateAt(burnUT + flightTime, out Vector3d arrived, out _)) continue;
                        double error = Vector3d.Distance(arrived, target);
                        double deltaV = (ToVector3d(transferVelocity) - velocity).magnitude;
                        bool reachesSurfaceAtTarget = transfer.TryNextRadiusCrossing(burnUT, snapshot.Body.Radius, out double crossingUT) &&
                            Math.Abs(crossingUT - (burnUT + flightTime)) < 1.0;
                        if (error < bestError)
                        {
                            bestError = error;
                            bestCrossingDifference = reachesSurfaceAtTarget ? crossingUT - (burnUT + flightTime) : double.NaN;
                        }
                        if (reachesSurfaceAtTarget && (double.IsNaN(bestDeltaV) || deltaV < bestDeltaV))
                        {
                            bestDeltaV = deltaV;
                            bestBurnUT = burnUT;
                            bestPosition = position;
                            bestVelocity = ToVector3d(transferVelocity);
                        }
                    }
                    catch (Exception) { }
                }
            }
            var transferSnapshot = new LandingGuidanceV2Snapshot(snapshot.Version, bestBurnUT, snapshot.Body,
                bestPosition, bestVelocity, snapshot.Mass, snapshot.AvailableDeltaV, snapshot.MaximumAcceleration,
                snapshot.MinimumAcceleration, snapshot.TargetLatitude, snapshot.TargetLongitude, false,
                snapshot.TargetReferenceUT, snapshot.TargetReferencePosition, snapshot.HasTargetReferencePosition);
            LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(transferSnapshot);
            Assert.True(bestError < 1.0 && Math.Abs(bestCrossingDifference) < 1.0 && bestDeltaV < snapshot.AvailableDeltaV && estimate.HasImpact,
                "bestError=" + bestError + " dv=" + bestDeltaV + " crossingDifference=" + bestCrossingDifference +
                " estimator=" + estimate.Outcome + " detail=" + estimate.Detail + " targetError=" + estimate.TargetError);
        }

        // Snapshot 71 from LandingGuidanceV2.trace.jsonl captured on 2026-09-22.
        // The target vector is the body-fixed vector recorded at targetReferenceUT.
        [Fact]
        public void RecordedMunSnapshotProducesAFeasibleLongSidePlan()
        {
            LandingGuidanceV2Snapshot snapshot = RecordedMunSnapshot();

            LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(snapshot);
            LandingGuidanceV2EstimatorValidation validation =
                AirlessImpactEstimator.ValidateDeterminism(snapshot, estimate);
            Vector3d horizontal = Vector3d.Exclude(snapshot.Position.normalized, snapshot.Velocity);
            var deorbit = new LandingGuidanceV2Snapshot(snapshot.Version, snapshot.UT, snapshot.Body,
                snapshot.Position, snapshot.Velocity - 100 * horizontal.normalized, snapshot.Mass,
                snapshot.AvailableDeltaV, snapshot.MaximumAcceleration, snapshot.MinimumAcceleration,
                snapshot.TargetLatitude, snapshot.TargetLongitude, false, snapshot.TargetReferenceUT,
                snapshot.TargetReferencePosition, snapshot.HasTargetReferencePosition);
            LandingGuidanceV2Estimate deorbitEstimate = AirlessImpactEstimator.Estimate(deorbit);
            AirlessLandingPlan plan = AirlessLandingPlanner.Plan(snapshot);

            Assert.True(validation.IsDeterministic);
            Assert.True(estimate.Outcome == LandingGuidanceV2EstimateOutcome.NoImpact, estimate.Detail);
            Assert.True(deorbitEstimate.HasImpact, deorbitEstimate.Detail);
            Assert.True(plan.State == AirlessLandingPlanState.Candidate,
                plan.Reason + " downrange=" + plan.SignedDownrange + " crossrange=" + plan.CrossRange +
                " corridor=" + plan.CorridorLimit + " dv=" + plan.StrategicDeorbitDeltaVMagnitude +
                " terminal=" + plan.TerminalBrakingLowerBound + " trim=" + plan.TrimBudget +
                " reserve=" + plan.TerminalDivertReserve +
                " margin=" + plan.LowerBoundMargin);
            Assert.True(IsFinite(plan.StrategicBurnUT));
            Assert.True(IsFinite(plan.BrakingEntryUT));
            Assert.True(double.IsNaN(plan.PlaneAlignmentBurnUT));
            Assert.True(plan.PlaneAlignmentDeltaV.magnitude <= 0.5);
            Assert.True(plan.StrategicDeorbitDeltaVMagnitude > 0);
            Assert.True(plan.CandidateEstimate.HasImpact);
            Assert.True(plan.SignedDownrange >= 0 && plan.SignedDownrange <= plan.CorridorLimit);
            Assert.True(plan.CrossRange <= plan.CorridorLimit);
            Assert.True(plan.LowerBoundMargin >= 0);
            Assert.True(plan.StrategicBurnUT >= snapshot.UT);
            Assert.True(plan.BrakingEntryUT > plan.StrategicBurnUT);
        }

        [Fact]
        public void EarlierRecordedMunSnapshotAlsoProducesAFeasiblePlan()
        {
            const double ut = 24108738.842233;
            var snapshot = new LandingGuidanceV2Snapshot(25, ut, CreateMun(),
                new Vector3d(237658.944818, -4540.145469, -24262.014012),
                new Vector3d(53.192611, 5.436655, 519.367374),
                36.077968, 810.440609, 27.717748, 0,
                0.165, -130.626389, false, ut,
                new Vector3d(-97232.142734, 575.957857, 174772.934666), true);
            AirlessLandingPlan plan = AirlessLandingPlanner.Plan(snapshot);
            Assert.True(plan.State == AirlessLandingPlanState.Candidate,
                plan.Reason + " downrange=" + plan.SignedDownrange + " crossrange=" + plan.CrossRange +
                " corridor=" + plan.CorridorLimit + " dv=" + plan.StrategicDeorbitDeltaVMagnitude);
        }

        [Fact]
        public void StartedV2MunSnapshotProducesAFeasiblePlan()
        {
            const double ut = 24108753.762232;
            var snapshot = new LandingGuidanceV2Snapshot(30, ut, CreateMun(),
                new Vector3d(220388.272710, -4456.631803, -92198.965400),
                new Vector3d(201.649583, 5.757169, 481.563639),
                36.077968, 810.440603, 27.717750, 0,
                0.165, -130.626389, false, ut,
                new Vector3d(-36056.402455, 575.957857, 196722.149527), true);
            AirlessLandingPlan plan = AirlessLandingPlanner.Plan(snapshot);
            Assert.True(plan.State == AirlessLandingPlanState.Candidate,
                plan.Reason + " downrange=" + plan.SignedDownrange + " crossrange=" + plan.CrossRange +
                " corridor=" + plan.CorridorLimit + " dv=" + plan.StrategicDeorbitDeltaVMagnitude);
        }

        private static LandingGuidanceV2Snapshot RecordedMunSnapshot()
        {
            const double ut = 24108762.642232;
            return new LandingGuidanceV2Snapshot(71, ut, CreateMun(),
                new Vector3d(238590.095384, -4404.672469, -12130.553663),
                new Vector3d(26.684653, 5.945045, 521.392817),
                36.077968, 810.440609, 27.717749, 0,
                0.165, -130.626389, false, ut,
                new Vector3d(-97232.142734, 575.957857, 174772.934666), true);
        }

        private static CelestialBody CreateMun()
        {
            var body = (CelestialBody)FormatterServices.GetUninitializedObject(typeof(CelestialBody));
            body.Radius = 200000;
            body.gravParameter = 6.5138398e10;
            body.atmosphere = false;
            body.rotationPeriod = 138984.38;
            body.angularVelocity = Vector3d.up * (2.0 * Math.PI / body.rotationPeriod);
            return body;
        }

        private static Vector3d RotateTarget(LandingGuidanceV2Snapshot snapshot, double ut)
        {
            Vector3d axis = snapshot.Body.angularVelocity.normalized;
            double radians = 2.0 * Math.PI * (ut - snapshot.TargetReferenceUT) / snapshot.Body.rotationPeriod;
            double c = Math.Cos(radians);
            double s = Math.Sin(radians);
            Vector3d target = snapshot.TargetReferencePosition;
            return target * c + Vector3d.Cross(axis, target) * s + axis * Vector3d.Dot(axis, target) * (1.0 - c);
        }

        private static V3 ToV3(Vector3d vector) => new V3(vector.x, vector.y, vector.z);
        private static Vector3d ToVector3d(V3 vector) => new Vector3d(vector.x, vector.y, vector.z);

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
