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
                        bool reachesSurfaceAtTarget = transfer.TryNextRadiusCrossing(burnUT, snapshot.Body.Radius + snapshot.TargetTerrainAltitude, out double crossingUT) &&
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
                snapshot.TargetReferenceUT, snapshot.TargetReferencePosition, snapshot.HasTargetReferencePosition, snapshot.TargetTerrainAltitude);
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
                new Vector3d(-97232.142734, 575.957857, 174772.934666), true, 4350.290494);
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
                new Vector3d(-36056.402455, 575.957857, 196722.149527), true, 4350.290494);
            AirlessLandingPlan plan = AirlessLandingPlanner.Plan(snapshot);
            Assert.True(plan.State == AirlessLandingPlanState.Candidate,
                plan.Reason + " downrange=" + plan.SignedDownrange + " crossrange=" + plan.CrossRange +
                " corridor=" + plan.CorridorLimit + " dv=" + plan.StrategicDeorbitDeltaVMagnitude);
        }

        [Fact]
        public void CommittedMunStrategicVectorRemainsValidAtItsFreshBurnGate()
        {
            LandingGuidanceV2Snapshot snapshot = RecordedMunSnapshot();
            AirlessLandingPlan planned = AirlessLandingPlanner.Plan(snapshot);
            Assert.True(planned.State == AirlessLandingPlanState.Candidate, planned.Reason);

            var coast = new AirlessConicTrajectory(snapshot.Body.gravParameter, snapshot.UT,
                snapshot.Position, snapshot.Velocity);
            Assert.True(coast.TryStateAt(planned.StrategicBurnUT, out Vector3d gatePosition, out Vector3d gateVelocity));
            Vector3d targetAtGate = RotateTarget(snapshot, planned.StrategicBurnUT);
            var gateSnapshot = new LandingGuidanceV2Snapshot(snapshot.Version + 1, planned.StrategicBurnUT,
                snapshot.Body, gatePosition, gateVelocity, snapshot.Mass, snapshot.AvailableDeltaV,
                snapshot.MaximumAcceleration, snapshot.MinimumAcceleration, snapshot.TargetLatitude,
                snapshot.TargetLongitude, false, planned.StrategicBurnUT, targetAtGate, true, 4350.290494);

            Assert.True(AirlessLandingPlanner.TryValidateCommittedStrategicBurn(gateSnapshot, planned,
                out AirlessLandingPlan validated, out string reason), reason);
            Assert.True(validated.State == AirlessLandingPlanState.Candidate);
            Assert.Equal(gateSnapshot.UT, validated.StrategicBurnUT, 6);
            Assert.True(validated.CandidateEstimate.HasImpact);
            Assert.True(validated.SignedDownrange >= 0 && validated.SignedDownrange <= validated.CorridorLimit);
            Assert.True(validated.CrossRange <= validated.CorridorLimit);
            Assert.True(validated.LowerBoundMargin >= 0);
        }

        [Fact]
        public void RecordedWarpPlanRemainsValidAtItsExactStrategicIgnitionUT()
        {
            const double ut = 24109518.539994;
            var snapshot = new LandingGuidanceV2Snapshot(717, ut, CreateMun(),
                new Vector3d(238650.736775, 3067.806340, -11602.139401),
                new Vector3d(25.191984, 9.112790, 521.391966),
                36.077968, 810.440756, 27.717764, 0,
                0.165, -130.626389, false, ut,
                new Vector3d(178892.060012, 575.957857, 89427.619543), true, 4350.290494);
            AirlessLandingPlan plan = AirlessLandingPlanner.Plan(snapshot);
            Assert.True(plan.State == AirlessLandingPlanState.Candidate, plan.Reason);

            var coast = new AirlessConicTrajectory(snapshot.Body.gravParameter, snapshot.UT,
                snapshot.Position, snapshot.Velocity);
            Assert.True(coast.TryStateAt(plan.StrategicBurnUT, out Vector3d gatePosition, out Vector3d gateVelocity));
            Vector3d targetAtGate = RotateTarget(snapshot, plan.StrategicBurnUT);
            var gate = new LandingGuidanceV2Snapshot(718, plan.StrategicBurnUT, snapshot.Body,
                gatePosition, gateVelocity, snapshot.Mass, snapshot.AvailableDeltaV,
                snapshot.MaximumAcceleration, snapshot.MinimumAcceleration, snapshot.TargetLatitude,
                snapshot.TargetLongitude, false, plan.StrategicBurnUT, targetAtGate, true, snapshot.TargetTerrainAltitude);

            Assert.True(AirlessLandingPlanner.TryValidateCommittedStrategicBurn(gate, plan,
                out AirlessLandingPlan validated, out string reason), reason);
            Assert.True(validated.CandidateEstimate.HasImpact);
            Assert.True(validated.SignedDownrange >= 0 && validated.SignedDownrange <= validated.CorridorLimit);
            Assert.True(validated.CrossRange <= validated.CorridorLimit);
            Assert.True(validated.LowerBoundMargin >= 0);
        }

        [Fact]
        public void LatestRecordedWarpPlanValidatesFiniteBurnFromItsIgnitionSnapshot()
        {
            const double ut = 24108735.542233;
            var snapshot = new LandingGuidanceV2Snapshot(18, ut, CreateMun(),
                new Vector3d(79042.051085, -4557.968156, -225438.468683),
                new Vector3d(492.736437, 5.364971, 172.580466),
                36.077968, 810.440682, 27.717754, 0,
                0.165, -130.626389, false, ut,
                new Vector3d(115731.229739, 575.957857, 163113.306433), true, 4350.290494);
            AirlessLandingPlan plan = AirlessLandingPlanner.Plan(snapshot);
            Assert.True(plan.State == AirlessLandingPlanState.Candidate, plan.Reason);

            var coast = new AirlessConicTrajectory(snapshot.Body.gravParameter, snapshot.UT,
                snapshot.Position, snapshot.Velocity);
            Assert.True(coast.TryStateAt(plan.StrategicBurnUT, out Vector3d position, out Vector3d velocity));
            var exactGate = new LandingGuidanceV2Snapshot(200, plan.StrategicBurnUT, snapshot.Body,
                position, velocity, snapshot.Mass, snapshot.AvailableDeltaV,
                snapshot.MaximumAcceleration, snapshot.MinimumAcceleration, snapshot.TargetLatitude,
                snapshot.TargetLongitude, false, plan.StrategicBurnUT, RotateTarget(snapshot, plan.StrategicBurnUT),
                true, snapshot.TargetTerrainAltitude);
            Assert.True(AirlessLandingPlanner.TryValidateCommittedStrategicBurn(exactGate, plan,
                out _, out string exactReason), exactReason);

            Assert.True(coast.TryStateAt(plan.StrategicBurnUT - 0.52, out position, out velocity));
            var earlyGate = new LandingGuidanceV2Snapshot(199, plan.StrategicBurnUT - 0.52, snapshot.Body,
                position, velocity, snapshot.Mass, snapshot.AvailableDeltaV,
                snapshot.MaximumAcceleration, snapshot.MinimumAcceleration, snapshot.TargetLatitude,
                snapshot.TargetLongitude, false, plan.StrategicBurnUT - 0.52,
                RotateTarget(snapshot, plan.StrategicBurnUT - 0.52), true, snapshot.TargetTerrainAltitude);
            // The fresh snapshot is taken at finite-burn ignition.  Validation
            // propagates it to the planned impulse midpoint, avoiding the old
            // false rejection caused by treating ignition as an early impulse.
            Assert.True(AirlessLandingPlanner.TryValidateCommittedStrategicBurn(earlyGate, plan,
                out _, out string ignitionReason), ignitionReason);
        }

        [Fact]
        public void FreshIgnitionSnapshotRetainsThePlanTargetReferenceEpoch()
        {
            const double planUT = 24108743.982232;
            var planSnapshot = new LandingGuidanceV2Snapshot(35, planUT, CreateMun(),
                new Vector3d(237759.381593, -4511.915183, -23266.167898),
                new Vector3d(51.017075, 5.547743, 519.583661),
                36.077968, 810.440638, 27.717748, 0,
                0.165, -130.626389, false, planUT,
                new Vector3d(-96039.528021, 575.957857, 175431.118477), true, 4350.290494);
            AirlessLandingPlan plan = AirlessLandingPlanner.Plan(planSnapshot);
            Assert.True(plan.State == AirlessLandingPlanState.Candidate, plan.Reason);

            const double ignitionUT = 24108864.742297;
            var gateSnapshot = new LandingGuidanceV2Snapshot(218, ignitionUT, planSnapshot.Body,
                new Vector3d(235831.355059, -3693.537018, 38271.602123),
                new Vector3d(-83.446804, 7.927263, 515.322565),
                36.077968, 810.440638, 27.717748, 0,
                0.165, -130.626389, false,
                // This is deliberately the plan epoch/vector, not a new
                // body-fixed value relabelled with ignition UT.
                planSnapshot.TargetReferenceUT, planSnapshot.TargetReferencePosition, true, 4350.290494);

            Assert.True(AirlessLandingPlanner.TryValidateCommittedStrategicBurn(gateSnapshot, plan,
                out AirlessLandingPlan validated, out string reason), reason);
            Assert.Equal(planSnapshot.TargetReferenceUT, gateSnapshot.TargetReferenceUT, 6);
            Assert.Equal(plan.StrategicBurnUT, validated.StrategicBurnUT, 6);
        }

        private static LandingGuidanceV2Snapshot RecordedMunSnapshot()
        {
            const double ut = 24108762.642232;
            return new LandingGuidanceV2Snapshot(71, ut, CreateMun(),
                new Vector3d(238590.095384, -4404.672469, -12130.553663),
                new Vector3d(26.684653, 5.945045, 521.392817),
                36.077968, 810.440609, 27.717749, 0,
                0.165, -130.626389, false, ut,
                new Vector3d(-97232.142734, 575.957857, 174772.934666), true, 4350.290494);
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
            Vector3d target = snapshot.TargetReferencePosition.normalized *
                (snapshot.Body.Radius + (double.IsNaN(snapshot.TargetTerrainAltitude) ? 0 : snapshot.TargetTerrainAltitude));
            return target * c + Vector3d.Cross(axis, target) * s + axis * Vector3d.Dot(axis, target) * (1.0 - c);
        }

        private static V3 ToV3(Vector3d vector) => new V3(vector.x, vector.y, vector.z);
        private static Vector3d ToVector3d(V3 vector) => new Vector3d(vector.x, vector.y, vector.z);

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
