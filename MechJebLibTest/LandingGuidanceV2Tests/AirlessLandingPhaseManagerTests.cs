using System;
using System.Runtime.Serialization;
using MuMech;
using UnityEngine;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class AirlessLandingPhaseManagerTests
    {
        [Fact]
        public void WarpIsNotAuthorizedUntilTheNextBurnAttitudeIsAlreadyReady()
        {
            var manager = new AirlessLandingPhaseManager();
            AirlessLandingPlan plan = Candidate(100, 1000, 1200, 30);
            Assert.Equal(AirlessLandingPhaseManagerPhase.InitialWarpToStrategicBurn, manager.Start(plan).Phase);
            Assert.Equal(AirlessLandingPhaseDirective.RequestInitialWarp, manager.Tick(100, true, 120, double.NaN).Directive);
            Assert.Equal(400, manager.Tick(100, true, 120, double.NaN).WarpUT, 6);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, manager.Tick(400, true, 120, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseManagerPhase.PrepareStrategicWarp, manager.Phase);
            Assert.Equal(AirlessLandingPhaseDirective.WarpAuthorized, manager.Tick(401, true, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseManagerPhase.WarpToStrategicBurn, manager.Phase);
            AirlessLandingPhaseDecision warp = manager.Tick(401.01, true, 90, double.NaN);
            Assert.Equal(AirlessLandingPhaseDirective.RequestWarp, warp.Directive);
            Assert.Equal(980, warp.WarpUT, 6);
        }

        [Fact]
        public void WarpExitStillRequiresFreshIgnitionValidationBeforeThrottle()
        {
            var manager = ReadyForStrategicWarp(1000, 30, 0.65);
            Assert.Equal(AirlessLandingPhaseDirective.RequestWarp, manager.Tick(900, true, 90, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.ExitWarpAndRequestAttitude, manager.Tick(979.35, true, 90, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, manager.Tick(999.34, false, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequireFreshStrategicValidation, manager.Tick(999.35, false, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, manager.AcceptStrategicValidation(101, 999.35, true).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.BeginFiniteBurn, manager.Tick(999.35, false, 0.1, 30).Directive);
        }

        [Fact]
        public void LateFiniteBurnGateFailsClosedBeforeAnyThrottleDirective()
        {
            var manager = ReadyForStrategicWarp(1000, 30, 0.65);
            manager.Tick(979.35, true, 90, double.NaN);
            Assert.Equal(AirlessLandingPhaseDirective.RequireFreshStrategicValidation,
                manager.Tick(999.35, false, 0.1, double.NaN).Directive);
            AirlessLandingPhaseDecision late = manager.AcceptStrategicValidation(101, 1000.26, true);
            Assert.Equal(AirlessLandingPhaseDirective.Reject, late.Directive);
            Assert.Contains("midpoint", late.Reason);
        }

        [Fact]
        public void FiniteBurnStopsThrottleOnMeasuredDeliveryAndNeverReopensIt()
        {
            var progress = new FiniteBurnProgress(3.0);
            // Reproduces the observed 27.9 m/sÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â² correction-burn condition.
            // Measured engine delivery reaches the planned value in six 20 ms
            // frames; 155 seconds of subsequent coast cannot make it burn again.
            for (int i = 0; i != 6; ++i) progress.Integrate(0.02, 27.9);
            Assert.True(progress.IsComplete());
            Assert.InRange(progress.DeliveredDeltaV, 3.0, 3.5);
            for (int i = 0; i != 7750; ++i) progress.Integrate(0.02, 0);
            Assert.True(progress.IsComplete());
            Assert.Equal(0, progress.RemainingDeltaV, 6);
        }

        [Fact]
        public void FiniteBurnCapsThrottleForTheLastHalfMetrePerSecond()
        {
            Assert.Equal(0.75f, FiniteBurnProgress.LimitThrottleForFineControl(0.501, 0.75f));
            Assert.Equal(FiniteBurnProgress.FineControlThrottleCap,
                FiniteBurnProgress.LimitThrottleForFineControl(0.50, 0.75f));
            Assert.Equal(0.01f, FiniteBurnProgress.LimitThrottleForFineControl(0.01, 0.01f));
        }

        [Fact]
        public void TerminalWarpRequiresTwoSecondsOfContinuousBrakingAttitude()
        {
            var gate = new AirlessTerminalWarpGate();
            Assert.False(gate.ObserveAttitude(100, 0.9));
            Assert.False(gate.ObserveAttitude(101.99, 0.9));
            Assert.True(gate.ObserveAttitude(102.0, 0.9));
            Assert.False(gate.ObserveAttitude(102.01, 1.01));
            Assert.False(gate.ObserveAttitude(103.0, 0.9));
            Assert.True(gate.ObserveAttitude(105.0, 0.9));
        }

        [Fact]
        public void FiniteBurnRequiresMeasuredThrustVectorAsWellAsControllerAlignment()
        {
            Assert.True(AirlessBurnAlignmentGate.IsReady(0.9, 0.9));
            Assert.False(AirlessBurnAlignmentGate.IsReady(0.9, 1.1));
            Assert.False(AirlessBurnAlignmentGate.IsReady(1.1, 0.9));
            Assert.Equal(1.1, AirlessBurnAlignmentGate.CombinedError(0.9, 1.1), 6);
            Assert.True(double.IsPositiveInfinity(AirlessBurnAlignmentGate.CombinedError(double.NaN, 0.1)));
        }

        [Fact]
        public void FiniteBurnRequiresTheAttitudeToBeSettled()
        {
            Assert.True(AirlessBurnAlignmentGate.IsReady(1.0, 1.0, 0.001));
            Assert.False(AirlessBurnAlignmentGate.IsReady(1.0, 1.0, 0.00101));
        }

        [Fact]
        public void ControllerHarnessExecutesStagedWarpFreshValidationAndMeasuredBurn()
        {
            AirlessLandingPlan plan = Candidate(100, 1200, 1500, 30);
            var result = new AirlessLandingControllerHarness().Execute(plan, 100, 0.65, 27.9);
            Assert.True(result.InitialWarpRequested);
            Assert.True(result.FinalWarpRequested);
            Assert.True(result.FreshValidationRequired);
            Assert.True(result.FiniteBurnStarted);
            Assert.True(result.FiniteBurnCompleted);
            Assert.Equal(AirlessLandingPhaseManagerPhase.Coast, result.FinalPhase);
            Assert.InRange(result.DeliveredDeltaV, 29.9, 30.6);
            Assert.Contains(AirlessLandingPhaseDirective.RequestAttitude, result.Directives);
            Assert.Contains(AirlessLandingPhaseDirective.RequestFiniteBurnThrottle, result.Directives);
            Assert.True(result.WorkUnits < 2000);
        }

        [Fact]
        public void ControllerHarnessExecutesPlaneAlignmentFreshReplanAndStrategicBurn()
        {
            AirlessLandingPlan planePlan = Candidate(100, 1400, 1700, 30, 8, 900);
            AirlessLandingPlan strategicReplan = Candidate(101, 1400, 1700, 28);
            var result = new AirlessLandingControllerHarness().Execute(planePlan, 100, 0.65, 27.9,
                strategicReplan: strategicReplan);
            Assert.True(result.InitialWarpRequested);
            Assert.True(result.FinalWarpRequested);
            Assert.True(result.FreshPlaneValidationRequired);
            Assert.True(result.PlaneBurnStarted);
            Assert.True(result.PlaneBurnCompleted);
            Assert.True(result.FreshValidationRequired);
            Assert.True(result.FiniteBurnStarted);
            Assert.True(result.FiniteBurnCompleted);
            Assert.Equal(AirlessLandingPhaseManagerPhase.Coast, result.FinalPhase);
            Assert.InRange(result.PlaneDeliveredDeltaV, 7.99, 8.6);
            Assert.InRange(result.DeliveredDeltaV, 27.99, 28.6);
            Assert.Contains(AirlessLandingPhaseDirective.RequireStrategicReplan, result.Directives);
        }

        [Fact]
        public void ControllerHarnessDeniesBothWarpsUntilMeasuredThrustVectorIsSettled()
        {
            AirlessLandingPlan plan = Candidate(100, 1200, 1500, 30);
            var result = new AirlessLandingControllerHarness().Execute(plan, 100, 0.65, 27.9,
                thrustVectorErrorDegrees: 1.01, angularVelocityRadiansPerSecond: 0.0005);
            Assert.True(result.InitialWarpRequested);
            Assert.False(result.FinalWarpRequested);
            Assert.False(result.FreshValidationRequired);
            Assert.False(result.FiniteBurnStarted);
            Assert.False(result.FiniteBurnCompleted);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, result.LastDirective);
            Assert.DoesNotContain(AirlessLandingPhaseDirective.RequestFiniteBurnThrottle, result.Directives);
        }

        [Fact]
        public void ControllerHarnessFailsClosedWhenFreshIgnitionValidationRejects()
        {
            AirlessLandingPlan plan = Candidate(100, 1200, 1500, 30);
            var result = new AirlessLandingControllerHarness().Execute(plan, 100, 0.65, 27.9,
                freshValidationIsValid: false);
            Assert.True(result.InitialWarpRequested);
            Assert.True(result.FinalWarpRequested);
            Assert.True(result.FreshValidationRequired);
            Assert.False(result.FiniteBurnStarted);
            Assert.False(result.FiniteBurnCompleted);
            Assert.Equal(AirlessLandingPhaseManagerPhase.Rejected, result.FinalPhase);
            Assert.Equal(AirlessLandingPhaseDirective.Reject, result.LastDirective);
            Assert.Contains("fresh ignition snapshot", result.LastReason);
            Assert.DoesNotContain(AirlessLandingPhaseDirective.RequestFiniteBurnThrottle, result.Directives);
        }

        [Fact]
        public void OneDegreeAuthorityGateDeniesFinalWarpUntilPhysicalAttitudeIsReady()
        {
            var manager = new AirlessLandingPhaseManager();
            AirlessLandingPlan plan = Candidate(100, 1200, 1500, 30);
            manager.Start(plan, 0.65);
            Assert.Equal(AirlessLandingPhaseDirective.RequestInitialWarp,
                manager.Tick(100, true, 90, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude,
                manager.Tick(600, true, 90, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude,
                manager.Tick(601, true, 1.01, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.WarpAuthorized,
                manager.Tick(602, true, 1.0, double.NaN).Directive);
        }

        [Fact]
        public void FiniteBurnDropsThrottleWhenAttitudeLeavesTheAuthorityGate()
        {
            var manager = ReadyForStrategicWarp(1000, 30, 0);
            manager.Tick(980, true, 90, double.NaN);
            manager.Tick(1000, true, 0.1, double.NaN);
            manager.AcceptStrategicValidation(101, 1000, true);
            Assert.Equal(AirlessLandingPhaseDirective.BeginFiniteBurn, manager.Tick(1000, true, 0.1, 30).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, manager.Tick(1000.1, true, 1.01, 20).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestFiniteBurnThrottle, manager.Tick(1000.2, true, 0.1, 20).Directive);
        }

        [Fact]
        public void PlaneBurnAlsoRequiresAttitudeBeforeWarpAndAChronologicalReplan()
        {
            var manager = new AirlessLandingPhaseManager();
            AirlessLandingPlan planePlan = Candidate(100, 1000, 1200, 30, 8, 900);
            Assert.Equal(AirlessLandingPhaseManagerPhase.InitialWarpToPlaneAlignment, manager.Start(planePlan).Phase);
            Assert.Equal(AirlessLandingPhaseDirective.RequestInitialWarp, manager.Tick(100, true, 30, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, manager.Tick(300, true, 30, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.WarpAuthorized, manager.Tick(851, true, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.ExitWarpAndRequestAttitude, manager.Tick(880, true, 90, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequireFreshPlaneValidation, manager.Tick(900, true, 0.1, 8).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, manager.AcceptPlaneAlignmentValidation(101, 900, true).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.BeginFiniteBurn, manager.Tick(900, true, 0.1, 8).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestFiniteBurnThrottle, manager.Tick(900.99, true, 0.1, 0.011).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.FiniteBurnComplete, manager.Tick(901, true, 0.1, 0.009).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequireStrategicReplan, manager.Tick(902, true, 0.1, 0).Directive);
            AirlessLandingPhaseDecision replan = manager.AdoptStrategicReplan(Candidate(102, 1100, 1300, 28));
            Assert.Equal(AirlessLandingPhaseManagerPhase.InitialWarpToStrategicBurn, replan.Phase);
        }

        [Fact]
        public void PlaneAlignmentRejectsAStaleOrInvalidPostWarpSnapshotBeforeThrottle()
        {
            var manager = new AirlessLandingPhaseManager();
            AirlessLandingPlan plan = Candidate(100, 1200, 1500, 30, 8, 900);
            manager.Start(plan, 0);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude,
                manager.Tick(900, false, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.WarpAuthorized,
                manager.Tick(900, false, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.ExitWarpAndRequestAttitude,
                manager.Tick(900, false, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequireFreshPlaneValidation,
                manager.Tick(900, false, 0.1, double.NaN).Directive);
            AirlessLandingPhaseDecision rejected = manager.AcceptPlaneAlignmentValidation(101, 900, false,
                "Plane vector is stale after warp.");
            Assert.Equal(AirlessLandingPhaseDirective.Reject, rejected.Directive);
            Assert.Contains("stale", rejected.Reason);
            Assert.Equal(AirlessLandingPhaseManagerPhase.Rejected, manager.Phase);
        }

        [Fact]
        public void EveryPhaseTickHasOneBoundedWorkUnit()
        {
            var manager = new AirlessLandingPhaseManager();
            manager.Start(Candidate(100, 1000, 1200, 30));
            for (int i = 0; i != 1000; ++i)
                Assert.Equal(1, manager.Tick(900 + i * 0.01, true, 30, 30).WorkUnits);
        }

        [Fact]
        public void InvalidPlansFailClosedWithoutWarpOrThrottle()
        {
            var manager = new AirlessLandingPhaseManager();
            AirlessLandingPhaseDecision decision = manager.Start(Candidate(100, 1000, 1200, 30, 0, double.NaN, -1));
            Assert.Equal(AirlessLandingPhaseDirective.Reject, decision.Directive);
            Assert.Contains("margin", decision.Reason);
        }

        private static AirlessLandingPhaseManager ReadyForStrategicWarp(double strategicUT, double strategicDv, double lead)
        {
            var manager = new AirlessLandingPhaseManager();
            manager.Start(Candidate(100, strategicUT, strategicUT + 200, strategicDv), lead);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, manager.Tick(800, true, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.WarpAuthorized, manager.Tick(800.01, true, 0.1, double.NaN).Directive);
            return manager;
        }

        private static AirlessLandingPlan Candidate(long version, double strategicUT, double brakingUT, double strategicDv,
            double planeDv = 0, double planeUT = double.NaN, double margin = 100)
        {
            return new AirlessLandingPlan(version, AirlessLandingPlanState.Candidate, new Vector3d(strategicDv, 0, 0),
                700, 200, 50, 400,
                new LandingGuidanceV2Estimate(version, LandingGuidanceV2EstimateOutcome.Impact, brakingUT + 50,
                    Vector3d.right * 200000, Vector3d.zero, 0, "test"),
                strategicDv + planeDv + 700 + 5 + 50 + 5 + margin, "test", strategicUT, 5, 50, 5,
                new Vector3d(planeDv, 0, 0), planeUT, brakingUT);
        }
    }
}
