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
            // Reproduces the observed 27.9 m/sÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â² correction-burn condition.
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
        public void FiniteBurnRemapsTheLastHalfMetrePerSecondThroughTheEngineThrustLimiter()
        {
            AirlessFineThrustCommand outsideFineRange = AirlessFineThrustControl.Calculate(0.501, 0.75, 0, 100);
            Assert.False(outsideFineRange.UseEngineThrustLimiter);
            Assert.Equal(0.75, outsideFineRange.RequestedThrottle, 12);

            AirlessFineThrustCommand command = AirlessFineThrustControl.Calculate(0.50, 0.75, 0, 100);
            Assert.True(command.UseEngineThrustLimiter);
            Assert.Equal(0.04, command.RelativeEngineThrustLimit, 6);
            Assert.Equal(0.5, command.RequestedThrottle, 12);
            Assert.Equal(2.0, command.ExpectedAcceleration, 12);
            Assert.Equal(command.ExpectedAcceleration,
                100 * command.RelativeEngineThrustLimit * command.RequestedThrottle, 6);
        }

        [Fact]
        public void FineThrustLimiterPreservesAnEngineMinimumThrustFloor()
        {
            AirlessFineThrustCommand command = AirlessFineThrustControl.Calculate(0.10, 0.75, 20, 100);
            Assert.True(command.UseEngineThrustLimiter);
            Assert.Equal(0.04, command.RelativeEngineThrustLimit, 6);
            Assert.Equal(0.5, command.RequestedThrottle, 12);
            Assert.Equal(21.6, command.ExpectedAcceleration, 12);
            Assert.Equal(command.ExpectedAcceleration,
                20 + (100 - 20) * command.RelativeEngineThrustLimit * command.RequestedThrottle, 6);
        }

        [Fact]
        public void FiniteBurnFineThrustRespectsTheMainThrottleSafetyCap()
        {
            // At 2% desired physical throttle, a 2% main-throttle cap must
            // use a full engine range instead of silently clipping V2's
            // normal 50% remapped command.
            AirlessFineThrustCommand command = AirlessFineThrustControl.Calculate(0.10, 0.75, 0, 100,
                availableMainThrottle: 0.02);
            Assert.True(command.UseEngineThrustLimiter);
            Assert.Equal(1.0, command.RelativeEngineThrustLimit, 12);
            Assert.Equal(0.02, command.RequestedThrottle, 12);
            Assert.Equal(2.0, command.ExpectedAcceleration, 12);
        }

        [Fact]
        public void FineThrustLimiterUsesTheExistingEngineLimitAsThePhysicalFullThrottleRange()
        {
            // An engine with 20 m/s² minimum acceleration and a pre-existing
            // 50% engine limiter has a 60 m/s² full-throttle result. V2 must
            // preserve that player-selected limit, then temporarily scale it
            // to 2% for the final burn: 20 + (100 - 20) * .02 * .5 = 20.8.
            AirlessFineThrustCommand command = AirlessFineThrustControl.Calculate(0.10, 0.75, 20, 60);
            Assert.True(command.UseEngineThrustLimiter);
            Assert.Equal(0.04, command.RelativeEngineThrustLimit, 6);
            Assert.Equal(0.5, command.RequestedThrottle, 12);
            Assert.Equal(20.8, command.ExpectedAcceleration, 12);
            Assert.Equal(command.ExpectedAcceleration,
                20 + (100 - 20) * (0.50 * command.RelativeEngineThrustLimit) * command.RequestedThrottle, 6);
        }

        [Fact]
        public void FineThrustLimiterDoesNotPretendAThrottleLockedEngineHasFineControl()
        {
            AirlessFineThrustCommand command = AirlessFineThrustControl.Calculate(0.10, 0.75, 100, 100);
            Assert.False(command.UseEngineThrustLimiter);
            Assert.Equal(0.75, command.RequestedThrottle, 12);
        }

        [Fact]
        public void TerminalFineThrustRemapsAContinuousLowThrottleWithoutChangingPhysicalAcceleration()
        {
            AirlessFineThrustCommand command = AirlessFineThrustControl.CalculateTerminal(0.054, 0, 30);
            Assert.True(command.UseEngineThrustLimiter);
            Assert.Equal(0.108, command.RelativeEngineThrustLimit, 12);
            Assert.Equal(0.5, command.RequestedThrottle, 12);
            Assert.Equal(1.62, command.ExpectedAcceleration, 12);
            Assert.Equal(command.ExpectedAcceleration,
                30 * command.RelativeEngineThrustLimit * command.RequestedThrottle, 12);
        }

        [Fact]
        public void TerminalFineThrustRespectsTheMainThrottleSafetyCapWithoutChangingPhysicalAcceleration()
        {
            // V2 must account for MechJeb's separate main-throttle cap. A
            // 10% cap still permits a 5.4% physical request by choosing a
            // 54% engine range and issuing 10% main throttle.
            AirlessFineThrustCommand command = AirlessFineThrustControl.CalculateTerminal(0.054, 0, 30,
                availableMainThrottle: 0.10);
            Assert.True(command.UseEngineThrustLimiter);
            Assert.Equal(0.54, command.RelativeEngineThrustLimit, 12);
            Assert.Equal(0.10, command.RequestedThrottle, 12);
            Assert.Equal(1.62, command.ExpectedAcceleration, 12);
        }

        [Fact]
        public void TerminalPolicyUsesOnlyAccelerationAvailableThroughTheMainThrottleSafetyCap()
        {
            AirlessTerminalGuidanceCommand command = AirlessTerminalGuidance.Calculate(Vector3d.zero,
                new Vector3d(0, -0.5, 0), Vector3d.up, 100, 1.63, 0, 30, 0.5, 0.10);
            Assert.True(command.Valid);
            Assert.InRange(command.RequestedThrottle, 0, 0.10);
            Assert.InRange(command.DesiredAcceleration, 0, 3.0);
        }

        [Fact]
        public void TerminalPolicyRejectsAMinimumThrustProfileMismatch()
        {
            AirlessTerminalGuidanceCommand command = AirlessTerminalGuidance.Calculate(Vector3d.zero,
                new Vector3d(0, -0.5, 0), Vector3d.up, 100, 1.63, 20, 100, 0.5);
            Assert.False(command.Valid);
            Assert.Contains("minimum continuous thrust", command.RejectionReason);
        }

        [Fact]
        public void VisualRebaseGateNeverConcealsAMaterialTargetingError()
        {
            Assert.Equal(VisualTargetAction.Rebase, VisualRebaseGate.Decide(500, 500));
            Assert.Equal(VisualTargetAction.RetainOriginalAndReportFailure, VisualRebaseGate.Decide(500.01, 500));
            Assert.Equal(VisualTargetAction.RetainOriginalAndReportFailure,
                VisualRebaseGate.Decide(double.NaN, 500));
        }

        [Fact]
        public void CoastSafetyReplansAStaleOrOutOfCorridorEndpointBeforeTheBrakingLead()
        {
            LandingGuidanceV2Snapshot snapshot = Snapshot(100, 1000, 20);
            LandingGuidanceV2Estimate outside = Impact(100, 1200, 900);
            Assert.Equal(AirlessCoastSafetyAction.Replan,
                AirlessCoastSafetyGate.Decide(snapshot, outside, 400, 200, false));
            Assert.Equal(AirlessCoastSafetyAction.Replan,
                AirlessCoastSafetyGate.Decide(snapshot, null, 400, 200, true));
        }

        [Fact]
        public void CoastSafetyEntersControlledBrakingInsideTheVehicleDerivedLead()
        {
            LandingGuidanceV2Snapshot snapshot = Snapshot(100, 100, 20);
            LandingGuidanceV2Estimate outside = Impact(100, 110, 900);
            // 200 m/s / 20 m/s^2 plus two seconds is a 12 second lead.
            Assert.Equal(AirlessCoastSafetyAction.EmergencyBrake,
                AirlessCoastSafetyGate.Decide(snapshot, outside, 400, 200, false));
        }

        [Fact]
        public void CommittedDescentCannotReturnToStrategicReplanAuthority()
        {
            Assert.Equal(CommittedAirlessDescentRecoveryAction.StrategicReplan,
                CommittedAirlessDescentRecoveryGate.Decide(false));
            Assert.Equal(CommittedAirlessDescentRecoveryAction.ControlledCoast,
                CommittedAirlessDescentRecoveryGate.Decide(true));
            Assert.True(CommittedAirlessDescentRecoveryGate.RequiresTerminalAlignmentReset(false));
            Assert.False(CommittedAirlessDescentRecoveryGate.RequiresTerminalAlignmentReset(true));
        }

        [Fact]
        public void TerminalPreWarpAlignmentLatchesTheFirstFiniteInertialAttitude()
        {
            var latch = new AirlessTerminalAttitudeLatch();
            Assert.True(latch.TryLatch(new Vector3d(0, 3, 4)));
            Assert.True(latch.IsLatched);
            Assert.InRange(Vector3d.Distance(latch.Attitude, new Vector3d(0, 0.6, 0.8)), 0, 1e-12);
            Assert.True(latch.TryLatch(new Vector3d(1, 0, 0)));
            Assert.InRange(Vector3d.Distance(latch.Attitude, new Vector3d(0, 0.6, 0.8)), 0, 1e-12);
            latch.Reset();
            Assert.False(latch.IsLatched);
            Assert.False(latch.TryLatch(new Vector3d(double.NaN, 0, 0)));
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
        public void ControllerHarnessCompletesTheSameFiniteAirlessBurnWithoutAutoWarp()
        {
            AirlessLandingPlan plan = Candidate(100, 1200, 1500, 30);
            AirlessLandingControllerHarnessResult warped = new AirlessLandingControllerHarness().Execute(
                plan, 100, 0.65, 27.9, autoWarp: true);
            AirlessLandingControllerHarnessResult unwarped = new AirlessLandingControllerHarness().Execute(
                plan, 100, 0.65, 27.9, autoWarp: false);

            Assert.True(unwarped.FreshValidationRequired);
            Assert.True(unwarped.FiniteBurnCompleted);
            Assert.False(unwarped.InitialWarpRequested);
            Assert.False(unwarped.FinalWarpRequested);
            Assert.InRange(System.Math.Abs(unwarped.DeliveredDeltaV - warped.DeliveredDeltaV), 0, 0.01);
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
            Assert.Equal(AirlessLandingPhaseDirective.FiniteBurnComplete, manager.Tick(901, true, 0.1, 0.004).Directive);
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
        public void AtmosphericEntryWarpRequiresASettledBurnAttitudeBeforeBothWarpGates()
        {
            const double burnUT = 1000;
            Assert.False(AtmosphericBurnWarpGate.CanRequestWarp(100, burnUT,
                AirlessLandingPhaseManager.InitialWarpLeadSeconds, true, false));
            Assert.True(AtmosphericBurnWarpGate.CanRequestWarp(100, burnUT,
                AirlessLandingPhaseManager.InitialWarpLeadSeconds, true, true));
            Assert.False(AtmosphericBurnWarpGate.CanRequestWarp(990, burnUT,
                AirlessLandingPhaseManager.WarpSettleMargin, true, true));
            Assert.True(AtmosphericBurnWarpGate.CanRequestWarp(979, burnUT,
                AirlessLandingPhaseManager.WarpSettleMargin, true, true));
        }

        [Fact]
        public void InvalidPlansFailClosedWithoutWarpOrThrottle()
        {
            var manager = new AirlessLandingPhaseManager();
            AirlessLandingPhaseDecision decision = manager.Start(Candidate(100, 1000, 1200, 30, 0, double.NaN, -1));
            Assert.Equal(AirlessLandingPhaseDirective.Reject, decision.Directive);
            Assert.Contains("margin", decision.Reason);
        }

        [Fact]
        public void TerminalPolicyRejectsAConfigurationThatCannotHover()
        {
            AirlessTerminalGuidanceCommand command = AirlessTerminalGuidance.Calculate(Vector3d.zero,
                Vector3d.zero, Vector3d.up, 100, 1.63, 0, 1.63, 0.5);
            Assert.False(command.Valid);
            Assert.Contains("greater than local gravity", command.RejectionReason);
        }

        [Fact]
        public void TerminalPolicyCommandsUpwardThrustAndHorizontalTargetCorrection()
        {
            AirlessTerminalGuidanceCommand command = AirlessTerminalGuidance.Calculate(
                new Vector3d(-600, 0, -1000), new Vector3d(0, 0, -80), Vector3d.up,
                1000, 1.63, 0, 30, 0.5);
            Assert.True(command.Valid);
            Assert.True(command.DesiredAcceleration > 1.63);
            Assert.True(command.ThrustDirection.y > 0);
            Assert.True(command.ThrustDirection.x < 0);
            Assert.InRange(command.RequestedThrottle, 0, 1);
        }

        [Fact]
        public void TerminalPolicyNumericalDescentReachesTheTargetAtSafeSpeed()
        {
            // A deterministic local-flight integration of the exact terminal
            // command policy used by the V2 module. This is intentionally not
            // a planner-only test: it verifies that a post-strategic state
            // with a substantial horizontal miss can brake, translate, and
            // reach terrain without a late uncontrolled impact.
            SimulateTerminalDescent(600, 1000, -80, 1.63, 0, 30, out bool touchedDown, out Vector3d position, out Vector3d velocity);
            Assert.True(touchedDown);
            Assert.InRange(Math.Abs(position.x), 0, 10);
            Assert.InRange(Math.Abs(velocity.x), 0, 3.0);
            Assert.InRange(Math.Abs(velocity.y), 0, 2.0);
        }

        [Fact]
        public void EndToEndHarnessReachesTerminalTouchdownAfterAStagedStrategicBurn()
        {
            AirlessLandingPlan plan = Candidate(100, 1200, 1500, 30);
            AirlessLandingControllerHarnessResult strategic =
                new AirlessLandingControllerHarness().Execute(plan, 100, 0.65, 27.9);
            Assert.True(strategic.InitialWarpRequested);
            Assert.True(strategic.FinalWarpRequested);
            Assert.True(strategic.FreshValidationRequired);
            Assert.True(strategic.FiniteBurnCompleted);
            Assert.Equal(AirlessLandingPhaseManagerPhase.Coast, strategic.FinalPhase);

            SimulateTerminalDescent(600, 1000, -80, 1.63, 0, 30, out bool touchedDown, out Vector3d position, out Vector3d velocity);
            Assert.True(touchedDown);
            Assert.InRange(Math.Abs(position.x), 0, 10);
            Assert.InRange(Math.Abs(velocity.x), 0, 3.0);
            Assert.InRange(Math.Abs(velocity.y), 0, 2.0);
        }

        [Theory]
        [InlineData(0.49, 0.0, 10.0, 300.0, 800.0, -45.0)]
        [InlineData(1.63, 0.0, 30.0, 600.0, 1000.0, -80.0)]
        [InlineData(1.63, 1.0, 30.0, 600.0, 1000.0, -80.0)]
        [InlineData(7.85, 0.0, 45.0, 500.0, 1200.0, -90.0)]
        public void TerminalPolicyNumericalDescentHandlesGenericAirlessGravityAndThrust(double gravity,
            double minimumAcceleration, double maximumAcceleration, double lateralError, double altitude, double verticalVelocity)
        {
            SimulateTerminalDescent(lateralError, altitude, verticalVelocity, gravity, minimumAcceleration, maximumAcceleration,
                out bool touchedDown, out Vector3d position, out Vector3d velocity);
            Assert.True(touchedDown);
            Assert.InRange(Math.Abs(position.x), 0, 15);
            Assert.InRange(Math.Abs(velocity.x), 0, 3.0);
            Assert.InRange(Math.Abs(velocity.y), 0, 2.0);
        }

        [Fact]
        public void TerminalPolicyRejectsAThrottleCapThatCannotStopTheCurrentDescent()
        {
            // 10% of 30 m/s² produces only 1.37 m/s² of net upward braking
            // on the Mun. An 80 m/s descent needs more than 2 km to stop, so
            // a 1 km terminal state must not be presented as controllable.
            AirlessTerminalGuidanceCommand command = AirlessTerminalGuidance.Calculate(new Vector3d(-600, -1000, 0),
                new Vector3d(0, -80, 0), Vector3d.up, 1000, 1.63, 0, 30, 0.5, 0.10);
            Assert.False(command.Valid);
            Assert.Contains("cannot stop the current descent", command.RejectionReason);
        }

        private static LandingGuidanceV2Snapshot Snapshot(long version, double ut, double maximumAcceleration)
        {
            var body = (CelestialBody)FormatterServices.GetUninitializedObject(typeof(CelestialBody));
            body.Radius = 200000;
            body.gravParameter = 6.5138398e10;
            body.rotationPeriod = 138984.38;
            return new LandingGuidanceV2Snapshot(version, ut, body, Vector3d.right * 250000,
                Vector3d.forward * 500, 10, 1000, maximumAcceleration, 0,
                0, 0, false, ut, Vector3d.right * 200000, true, 0);
        }

        private static LandingGuidanceV2Estimate Impact(long version, double impactUT, double targetError) =>
            new LandingGuidanceV2Estimate(version, LandingGuidanceV2EstimateOutcome.Impact, impactUT,
                Vector3d.right * 200000, Vector3d.zero, targetError, "test");

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

        private static void SimulateTerminalDescent(double lateralError, double altitude, double verticalVelocity,
            double gravity, double minimumAcceleration, double maximumAcceleration, out bool touchedDown, out Vector3d position,
            out Vector3d velocity, double availableMainThrottle = 1.0)
        {
            position = new Vector3d(lateralError, altitude, 0);
            velocity = new Vector3d(0, verticalVelocity, 0);
            const double step = 0.02;
            touchedDown = false;
            for (int i = 0; i != 30000; ++i)
            {
                AirlessTerminalGuidanceCommand command = AirlessTerminalGuidance.Calculate(-position, velocity,
                    Vector3d.up, position.y, gravity, minimumAcceleration, maximumAcceleration, 0.5, availableMainThrottle);
                Assert.True(command.Valid, command.RejectionReason + " position=" + position + " velocity=" + velocity +
                    " gravity=" + gravity + " maxAcceleration=" + maximumAcceleration + " throttleLimit=" + availableMainThrottle);
                AirlessFineThrustCommand fineCommand = AirlessFineThrustControl.CalculateTerminal(command.RequestedThrottle,
                    minimumAcceleration, maximumAcceleration, availableMainThrottle: availableMainThrottle);
                Assert.InRange(fineCommand.RequestedThrottle, 0, availableMainThrottle);
                double deliveredAcceleration = minimumAcceleration + (maximumAcceleration - minimumAcceleration) *
                    fineCommand.RelativeEngineThrustLimit * fineCommand.RequestedThrottle;
                velocity += (command.ThrustDirection * deliveredAcceleration - Vector3d.up * gravity) * step;
                position += velocity * step;
                if (position.y <= 0)
                {
                    touchedDown = true;
                    return;
                }
            }
        }
    }
}
