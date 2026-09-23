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
        public void RecordedMunPlanCompletesTheStagedAuthoritySequenceInTheOfflineController()
        {
            LandingGuidanceV2Snapshot snapshot = RecordedMunSnapshot();
            AirlessLandingPlan plan = AirlessLandingPlanner.Plan(snapshot);
            Assert.Equal(AirlessLandingPlanState.Candidate, plan.State);

            // The controller harness does not solve new guidance. It executes
            // the phase authority plan against the captured, feasible Mun
            // burn: staged warp, fresh ignition validation, then measured
            // engine delta-v completion.
            double finiteBurnLead = plan.StrategicDeorbitDeltaVMagnitude / snapshot.MaximumAcceleration * 0.5;
            var result = new AirlessLandingControllerHarness().Execute(plan, snapshot.UT,
                finiteBurnLead, snapshot.MaximumAcceleration);

            Assert.True(result.FreshValidationRequired);
            Assert.True(result.FiniteBurnStarted);
            Assert.True(result.FiniteBurnCompleted);
            Assert.Equal(AirlessLandingPhaseManagerPhase.Coast, result.FinalPhase);
            Assert.InRange(result.DeliveredDeltaV, plan.StrategicDeorbitDeltaVMagnitude - 0.01,
                plan.StrategicDeorbitDeltaVMagnitude + snapshot.MaximumAcceleration * 0.021);
            Assert.Contains(AirlessLandingPhaseDirective.RequestAttitude, result.Directives);
            Assert.Contains(AirlessLandingPhaseDirective.BeginFiniteBurn, result.Directives);
            Assert.Contains(AirlessLandingPhaseDirective.FiniteBurnComplete, result.Directives);
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
            var gateSnapshot = new LandingGuidanceV2Snapshot(snapshot.Version + 1, planned.StrategicBurnUT,
                snapshot.Body, gatePosition, gateVelocity, snapshot.Mass, snapshot.AvailableDeltaV,
                snapshot.MaximumAcceleration, snapshot.MinimumAcceleration, snapshot.TargetLatitude,
                snapshot.TargetLongitude, false, snapshot.TargetReferenceUT, snapshot.TargetReferencePosition, true, 4350.290494);

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
            var gate = new LandingGuidanceV2Snapshot(718, plan.StrategicBurnUT, snapshot.Body,
                gatePosition, gateVelocity, snapshot.Mass, snapshot.AvailableDeltaV,
                snapshot.MaximumAcceleration, snapshot.MinimumAcceleration, snapshot.TargetLatitude,
                snapshot.TargetLongitude, false, snapshot.TargetReferenceUT, snapshot.TargetReferencePosition, true, snapshot.TargetTerrainAltitude);

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
                snapshot.TargetLongitude, false, snapshot.TargetReferenceUT, snapshot.TargetReferencePosition,
                true, snapshot.TargetTerrainAltitude);
            Assert.True(AirlessLandingPlanner.TryValidateCommittedStrategicBurn(exactGate, plan,
                out _, out string exactReason), exactReason);

            Assert.True(coast.TryStateAt(plan.StrategicBurnUT - 0.52, out position, out velocity));
            var earlyGate = new LandingGuidanceV2Snapshot(199, plan.StrategicBurnUT - 0.52, snapshot.Body,
                position, velocity, snapshot.Mass, snapshot.AvailableDeltaV,
                snapshot.MaximumAcceleration, snapshot.MinimumAcceleration, snapshot.TargetLatitude,
                snapshot.TargetLongitude, false, snapshot.TargetReferenceUT,
                snapshot.TargetReferencePosition, true, snapshot.TargetTerrainAltitude);
            // The fresh snapshot is taken at finite-burn ignition.  Validation
            // propagates it to the planned impulse midpoint, avoiding the old
            // false rejection caused by treating ignition as an early impulse.
            Assert.True(AirlessLandingPlanner.TryValidateCommittedStrategicBurn(earlyGate, plan,
                out _, out string ignitionReason), ignitionReason);
        }

        [Fact]
        public void IgnitionGateRejectsAChangedTargetReferenceEpoch()
        {
            LandingGuidanceV2Snapshot snapshot = RecordedMunSnapshot();
            AirlessLandingPlan plan = AirlessLandingPlanner.Plan(snapshot);
            Assert.True(plan.State == AirlessLandingPlanState.Candidate, plan.Reason);
            var coast = new AirlessConicTrajectory(snapshot.Body.gravParameter, snapshot.UT, snapshot.Position, snapshot.Velocity);
            Assert.True(coast.TryStateAt(plan.StrategicBurnUT, out Vector3d position, out Vector3d velocity));
            var corrupted = new LandingGuidanceV2Snapshot(snapshot.Version + 1, plan.StrategicBurnUT, snapshot.Body,
                position, velocity, snapshot.Mass, snapshot.AvailableDeltaV, snapshot.MaximumAcceleration,
                snapshot.MinimumAcceleration, snapshot.TargetLatitude, snapshot.TargetLongitude, false,
                snapshot.TargetReferenceUT + 1.0, snapshot.TargetReferencePosition, true, snapshot.TargetTerrainAltitude);
            Assert.False(AirlessLandingPlanner.TryValidateCommittedStrategicBurn(corrupted, plan, out _, out string reason));
            Assert.Contains("immutable target reference", reason);
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

        [Fact]
        public void RecordedPostStrategicStateCanBeReplannedIntoTheTargetCorridor()
        {
            // Snapshot 206 from the failed 2026-09-23 run, immediately after
            // the finite strategic burn. It has a valid but 11 km-long miss.
            // A controller may not coast or warp this state unless a new
            // correction is independently proven to return to the corridor.
            var postStrategic = new LandingGuidanceV2Snapshot(206, 24108857.921943, CreateMun(),
                new Vector3d(-50579.221954, -3760.922914, -233496.709043),
                new Vector3d(483.843193, -1.407125, -102.351519),
                35.722613, 782.080494, 27.993481, 0, 0.165, -130.626389, false,
                24108734.642233, new Vector3d(198001.793865, 575.957857, 28194.997763), true, 4350.290494);
            LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(postStrategic);
            Assert.True(estimate.HasImpact);
            Assert.True(estimate.TargetError > 10000, "targetError=" + estimate.TargetError);
            AirlessLandingPlan replan = AirlessLandingPlanner.Plan(postStrategic);
            Assert.True(replan.State == AirlessLandingPlanState.Candidate, replan.Reason + " error=" + estimate.TargetError);
            Assert.True(replan.CandidateEstimate.HasImpact);
            Assert.True(replan.CandidateEstimate.TargetError <= replan.CorridorLimit,
                "error=" + replan.CandidateEstimate.TargetError + " corridor=" + replan.CorridorLimit);
            Assert.True(replan.LowerBoundMargin >= 0, "margin=" + replan.LowerBoundMargin);
            Assert.True(replan.StrategicBurnUT >= postStrategic.UT);
            Assert.True(replan.StrategicBurnUT < replan.CandidateEstimate.ImpactUT);
        }

        [Fact]
        public void RecordedPostStrategicRecoveryPlanPassesFreshFiniteBurnGate()
        {
            var postStrategic = new LandingGuidanceV2Snapshot(206, 24108857.921943, CreateMun(),
                new Vector3d(-50579.221954, -3760.922914, -233496.709043),
                new Vector3d(483.843193, -1.407125, -102.351519),
                35.722613, 782.080494, 27.993481, 0, 0.165, -130.626389, false,
                24108734.642233, new Vector3d(198001.793865, 575.957857, 28194.997763), true, 4350.290494);
            AirlessLandingPlan recovery = AirlessLandingPlanner.Plan(postStrategic);
            Assert.True(recovery.CommandAuthorized, recovery.Reason);

            double lead = recovery.StrategicDeorbitDeltaVMagnitude / postStrategic.MaximumAcceleration * 0.5 + 0.10;
            double ignitionUT = recovery.StrategicBurnUT - lead;
            Assert.True(ignitionUT >= postStrategic.UT, "ignition=" + ignitionUT + " start=" + postStrategic.UT);
            var coast = new AirlessConicTrajectory(postStrategic.Body.gravParameter, postStrategic.UT,
                postStrategic.Position, postStrategic.Velocity);
            Assert.True(coast.TryStateAt(ignitionUT, out Vector3d ignitionPosition, out Vector3d ignitionVelocity));
            var ignition = new LandingGuidanceV2Snapshot(207, ignitionUT, postStrategic.Body,
                ignitionPosition, ignitionVelocity, postStrategic.Mass, postStrategic.AvailableDeltaV,
                postStrategic.MaximumAcceleration, postStrategic.MinimumAcceleration, postStrategic.TargetLatitude,
                postStrategic.TargetLongitude, false, postStrategic.TargetReferenceUT,
                postStrategic.TargetReferencePosition, postStrategic.HasTargetReferencePosition,
                postStrategic.TargetTerrainAltitude);
            Assert.True(AirlessLandingPlanner.TryValidateCommittedStrategicBurn(ignition, recovery,
                out AirlessLandingPlan validated, out string reason), reason);
            Assert.True(validated.CandidateEstimate.TargetError <= validated.CorridorLimit);

            var manager = new AirlessLandingPhaseManager();
            manager.Start(validated, lead);
            // Warp is prohibited until the future burn attitude has already
            // converged at 1x. This recovery candidate is already too late
            // for rails, so the same gate advances directly to warp exit.
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude,
                manager.Tick(ignitionUT, true, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.WarpAuthorized,
                manager.Tick(ignitionUT, true, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.ExitWarpAndRequestAttitude,
                manager.Tick(ignitionUT, true, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequireFreshStrategicValidation,
                manager.Tick(ignitionUT, false, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude,
                manager.AcceptStrategicValidation(208, ignitionUT, true).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.BeginFiniteBurn,
                manager.Tick(ignitionUT, false, 0.1, validated.StrategicDeorbitDeltaVMagnitude).Directive);
        }

        [Fact]
        public void RecordedPostStrategicResidualUsesProtectedRecoveryTrim()
        {
            // Trace record 207 from the unsafe 2026-09-23 V2 run. The finite
            // strategic burn had completed, left a valid impact 3.4 km from
            // target, and still had 781 m/s available. This state must remain
            // under V2 terminal authority rather than releasing the vessel.
            var postStrategic = new LandingGuidanceV2Snapshot(207, 24108858.121950, CreateMun(),
                new Vector3d(236476.931661, -3764.218582, 34012.301134),
                new Vector3d(-73.017664, -1.625097, 488.587856),
                35.715261, 781.342521, 27.999233, 0, 0.165, -130.626389, false,
                24108734.322233, new Vector3d(-95401.528904, 575.957857, 175778.885408), true, 4350.290494);
            AirlessLandingPlan committed = AirlessLandingPlanner.Plan(RecordedMunSnapshot());
            Assert.True(committed.State == AirlessLandingPlanState.Candidate, committed.Reason);
            LandingGuidanceV2Estimate before = AirlessImpactEstimator.Estimate(postStrategic);
            Assert.True(before.HasImpact);
            Assert.True(before.TargetError > committed.CorridorLimit);

            // This snapshot is deliberately conservative enough that the
            // reserve calculation cannot prove a terminal allocation. The
            // required result is a complete fresh replan, never coast, warp,
            // or a rejected controller that releases the vessel.
            AirlessPostBurnDecision decision = AirlessLandingPlanner.DecidePostBurn(postStrategic, committed, true);
            Assert.Equal(AirlessPostBurnAction.Replan, decision.Action);
            Assert.True(decision.Estimate.HasImpact);
            Assert.Contains("fresh complete target-transfer plan", decision.Reason);
        }

        [Fact]
        public void RecordedTerminalWarpExitOutsideCorridorIsDenied()
        {
            // The 2026-09-23 V2 trace showed this state after terminal warp:
            // post-trim estimate was 265 m, but the fresh physical state had
            // drifted to a 7.45 km miss. Rails must never be authorized from
            // the old estimate.
            var snapshot = new LandingGuidanceV2Snapshot(767, 24109671.740519, CreateMun(),
                new Vector3d(-62332.156092, 162.143502, 196193.948137),
                new Vector3d(-530.060654, 9.255353, -224.960816),
                35.706558, 779.996775, 28.006067, 0, 0.165, -130.626389, false,
                24108734.462233, new Vector3d(-74229.781420, 575.957857, 185713.779303), true, 4350.290494);
            LandingGuidanceV2Estimate estimate = AirlessImpactEstimator.Estimate(snapshot);
            Assert.True(estimate.HasImpact, estimate.Detail);
            Assert.True(estimate.TargetError > 7000, "targetError=" + estimate.TargetError);
            Assert.False(AirlessTerminalWarpGate.EndpointIsCurrentAndWithinCorridor(estimate, 400));
        }

        [Fact]
        public void RecordedMunFiniteStrategicBurnReachesThePlannedImpactCorridor()
        {
            // This is a deterministic closed-loop burn execution, not an
            // impulse-only planner check. It coasts to finite-burn ignition,
            // applies the planned vector in 20 ms measured-thrust samples
            // through two-body propagation, then evaluates the actual state.
            LandingGuidanceV2Snapshot start = RecordedMunSnapshot();
            AirlessLandingPlan plan = AirlessLandingPlanner.Plan(start);
            Assert.True(plan.CommandAuthorized, plan.Reason);
            double duration = plan.StrategicDeorbitDeltaVMagnitude / start.MaximumAcceleration;
            double ignitionUT = plan.StrategicBurnUT - duration * 0.5;
            Assert.True(ignitionUT >= start.UT, "ignition=" + ignitionUT + " start=" + start.UT);

            var coast = new AirlessConicTrajectory(start.Body.gravParameter, start.UT, start.Position, start.Velocity);
            Assert.True(coast.TryStateAt(ignitionUT, out Vector3d position, out Vector3d velocity));
            Vector3d thrustDirection = plan.StrategicDeorbitDeltaV.normalized;
            double ut = ignitionUT;
            double elapsed = 0;
            var progress = new FiniteBurnProgress(plan.StrategicDeorbitDeltaVMagnitude);
            while (elapsed < duration)
            {
                double dt = Math.Min(0.02, duration - elapsed);
                var step = new AirlessConicTrajectory(start.Body.gravParameter, ut, position, velocity);
                Assert.True(step.TryStateAt(ut + dt, out position, out velocity));
                velocity += thrustDirection * (start.MaximumAcceleration * dt);
                progress.Integrate(dt, start.MaximumAcceleration);
                ut += dt;
                elapsed += dt;
            }
            Assert.True(progress.IsComplete(0.001));
            Assert.InRange(progress.RemainingDeltaV, 0, 0.001);

            var postBurn = new LandingGuidanceV2Snapshot(start.Version + 1, ut, start.Body, position, velocity,
                start.Mass, start.AvailableDeltaV - progress.DeliveredDeltaV, start.MaximumAcceleration,
                start.MinimumAcceleration, start.TargetLatitude, start.TargetLongitude, false,
                start.TargetReferenceUT, start.TargetReferencePosition, start.HasTargetReferencePosition,
                start.TargetTerrainAltitude);
            LandingGuidanceV2Estimate actual = AirlessImpactEstimator.Estimate(postBurn);
            Assert.True(actual.HasImpact, actual.Detail);
            Assert.True(actual.TargetError <= plan.CorridorLimit,
                "actualError=" + actual.TargetError + " corridor=" + plan.CorridorLimit +
                " duration=" + duration + " delivered=" + progress.DeliveredDeltaV);
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
