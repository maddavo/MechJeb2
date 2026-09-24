using System;
using MuMech;
using UnityEngine;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class AtmosphericEntryPhaseManagerTests
    {
        [Fact]
        public void CandidateEntryBurnUsesStagedWarpFreshValidationAndMeasuredFiniteCompletion()
        {
            var manager = new AtmosphericEntryPhaseManager(0.65);
            AtmosphericLandingPlan plan = Candidate(100, 1000, 30);
            Assert.Equal(AtmosphericEntryPhase.InitialWarpToBurn, manager.Start(plan).Phase);

            AtmosphericEntryPhaseDecision initialWarp = manager.Tick(100, true, 60, double.NaN);
            Assert.Equal(AtmosphericEntryDirective.RequestInitialWarp, initialWarp.Directive);
            Assert.Equal(399.35, initialWarp.WarpUT, 6);
            Assert.Equal(AtmosphericEntryDirective.RequestAttitude,
                manager.Tick(initialWarp.WarpUT, true, 60, double.NaN).Directive);
            Assert.Equal(AtmosphericEntryDirective.WarpAuthorized,
                manager.Tick(initialWarp.WarpUT + 0.01, true, 0.1, double.NaN).Directive);

            AtmosphericEntryPhaseDecision finalWarp = manager.Tick(initialWarp.WarpUT + 0.02, true, 20, double.NaN);
            Assert.Equal(AtmosphericEntryDirective.RequestWarp, finalWarp.Directive);
            Assert.Equal(979.35, finalWarp.WarpUT, 6);
            Assert.Equal(AtmosphericEntryDirective.ExitWarpAndRequestAttitude,
                manager.Tick(finalWarp.WarpUT, true, 20, double.NaN).Directive);
            Assert.Equal(AtmosphericEntryDirective.RequireFreshBurnValidation,
                manager.Tick(999.35, false, 0.1, double.NaN).Directive);
            Assert.Equal(AtmosphericEntryDirective.RequestAttitude,
                manager.AcceptFreshBurnValidation(101, 999.35, true).Directive);
            Assert.Equal(AtmosphericEntryDirective.BeginFiniteBurn,
                manager.Tick(999.35, false, 0.1, 30).Directive);

            var progress = new FiniteBurnProgress(30);
            double ut = 999.35;
            AtmosphericEntryPhaseDecision? decision = null;
            while (!progress.IsComplete() && ut < 1100)
            {
                double requestedThrottle = Math.Max(0.01, Math.Min(1.0, progress.RemainingDeltaV / (0.5 * 27.9)));
                AirlessFineThrustCommand command = AirlessFineThrustControl.Calculate(progress.RemainingDeltaV,
                    requestedThrottle, 0, 27.9);
                progress.Integrate(0.02, command.ExpectedAcceleration);
                ut += 0.02;
                decision = manager.Tick(ut, false, 0.1, progress.RemainingDeltaV);
                if (decision.Directive == AtmosphericEntryDirective.FiniteBurnComplete) break;
                Assert.Equal(AtmosphericEntryDirective.RequestFiniteBurnThrottle, decision.Directive);
            }

            Assert.NotNull(decision);
            Assert.Equal(AtmosphericEntryDirective.FiniteBurnComplete, decision.Directive);
            Assert.InRange(Math.Abs(progress.DeliveredDeltaV - 30), 0, AtmosphericEntryPhaseManager.BurnCompleteDeltaV + 0.001);
            Assert.Equal(AtmosphericEntryDirective.RequirePostBurnValidation,
                manager.Tick(ut, false, 0.1, progress.RemainingDeltaV).Directive);
            Assert.Equal(AtmosphericEntryDirective.EnterAtmosphericEntry,
                manager.AcceptPostBurnValidation(Candidate(102, ut, 0)).Directive);
            Assert.Equal(AtmosphericEntryPhase.Entry, manager.Phase);
        }

        [Fact]
        public void ControllerHarnessExecutesAtmosphericWarpBurnAndPostBurnEntryValidation()
        {
            AtmosphericLandingPlan plan = Candidate(100, 1000, 30);
            AtmosphericLandingPlan postBurn = Candidate(102, 1001, 0);
            AtmosphericEntryControllerHarnessResult result = new AtmosphericEntryControllerHarness().Execute(
                plan, 100, 0.65, 27.9, postBurn);

            Assert.True(result.InitialWarpRequested);
            Assert.True(result.FinalWarpRequested);
            Assert.True(result.FreshBurnValidationRequired);
            Assert.True(result.FiniteBurnStarted);
            Assert.True(result.NonzeroThrottleCommanded);
            Assert.True(result.FiniteBurnCompleted);
            Assert.True(result.PostBurnValidationRequired);
            Assert.Equal(AtmosphericEntryPhase.Entry, result.FinalPhase);
            Assert.InRange(System.Math.Abs(result.DeliveredDeltaV - 30), 0,
                AtmosphericEntryPhaseManager.BurnCompleteDeltaV + 0.001);
            Assert.True(result.WorkUnits < 10000);
        }

        [Fact]
        public void ControllerHarnessFailsClosedWhenFreshAtmosphericIgnitionValidationRejects()
        {
            AtmosphericEntryControllerHarnessResult result = new AtmosphericEntryControllerHarness().Execute(
                Candidate(100, 1000, 30), 100, 0.65, 27.9, Candidate(102, 1001, 0),
                freshValidationIsValid: false);

            Assert.Equal(AtmosphericEntryPhase.Rejected, result.FinalPhase);
            Assert.Contains("deliberately rejected", result.LastReason);
            Assert.False(result.FiniteBurnStarted);
        }

        [Fact]
        public void AtmosphericBurnCannotStartWithoutAFreshPostWarpValidation()
        {
            var manager = new AtmosphericEntryPhaseManager(0.65);
            manager.Start(Candidate(100, 1000, 30));
            manager.Tick(400, true, 30, double.NaN);
            manager.Tick(401, true, 0.1, double.NaN);
            manager.Tick(401.01, true, 0.1, double.NaN);
            manager.Tick(979.35, true, 0.1, double.NaN);
            Assert.Equal(AtmosphericEntryDirective.RequireFreshBurnValidation,
                manager.Tick(999.35, false, 0.1, double.NaN).Directive);
            Assert.Equal(AtmosphericEntryPhase.AlignBurn, manager.Phase);
        }

        [Fact]
        public void AtmosphericPostBurnValidationFailsClosedForAStaleOrUnfundedPlan()
        {
            var manager = ReadyForPostBurnValidation();
            AtmosphericEntryPhaseDecision stale = manager.AcceptPostBurnValidation(Candidate(100, 1001, 0));
            Assert.Equal(AtmosphericEntryDirective.Reject, stale.Directive);
            Assert.Contains("fresh snapshot", stale.Reason);

            manager = ReadyForPostBurnValidation();
            AtmosphericEntryPhaseDecision unfunded = manager.AcceptPostBurnValidation(Candidate(102, 1001, 0, -1));
            Assert.Equal(AtmosphericEntryDirective.Reject, unfunded.Directive);
            Assert.Contains("terminal reserve", unfunded.Reason);
        }



        [Fact]
        public void FreshIgnitionValidationMayEnterAValidatedDirectTrajectoryWithoutForcingAnObsoleteBurn()
        {
            var manager = new AtmosphericEntryPhaseManager(0.65);
            manager.Start(Candidate(100, 1000, 30));
            manager.Tick(400, true, 30, double.NaN);
            manager.Tick(401, true, 0.1, double.NaN);
            manager.Tick(401.01, true, 0.1, double.NaN);
            manager.Tick(979.35, true, 0.1, double.NaN);
            manager.Tick(999.35, false, 0.1, double.NaN);

            Assert.Equal(AtmosphericEntryDirective.EnterAtmosphericEntry,
                manager.AcceptFreshBurnValidation(Candidate(101, 999.35, 0), 999.35).Directive);
            Assert.Equal(AtmosphericEntryPhase.Entry, manager.Phase);
        }

        [Fact]
        public void FreshPostBurnCorrectionRestartsStagedBurnSequenceRatherThanEnteringUnguided()
        {
            var manager = ReadyForPostBurnValidation();
            AtmosphericEntryPhaseDecision correction = manager.AcceptPostBurnValidation(Candidate(102, 1400, 12));

            Assert.Equal(AtmosphericEntryDirective.None, correction.Directive);
            Assert.Equal(AtmosphericEntryPhase.InitialWarpToBurn, manager.Phase);
            Assert.Equal(AtmosphericEntryDirective.RequestAttitude,
                manager.Tick(1001, true, 10, double.NaN).Directive);
            Assert.Equal(AtmosphericEntryDirective.WarpAuthorized,
                manager.Tick(1001.01, true, 0.1, double.NaN).Directive);
            Assert.Equal(AtmosphericEntryDirective.RequestWarp,
                manager.Tick(1001.02, true, 0.1, double.NaN).Directive);
        }

        private static AtmosphericEntryPhaseManager ReadyForPostBurnValidation()
        {
            var manager = new AtmosphericEntryPhaseManager(0.65);
            manager.Start(Candidate(100, 1000, 30));
            manager.Tick(400, true, 30, double.NaN);
            manager.Tick(401, true, 0.1, double.NaN);
            manager.Tick(401.01, true, 0.1, double.NaN);
            manager.Tick(979.35, true, 0.1, double.NaN);
            manager.Tick(999.35, false, 0.1, double.NaN);
            manager.AcceptFreshBurnValidation(101, 999.35, true);
            manager.Tick(999.35, false, 0.1, 30);
            Assert.Equal(AtmosphericEntryDirective.FiniteBurnComplete,
                manager.Tick(1000, false, 0.1, 0.001).Directive);
            return manager;
        }

        private static AtmosphericLandingPlan Candidate(long version, double burnUT, double deltaV, double margin = 100) =>
            new AtmosphericLandingPlan(version, AtmosphericLandingPlanState.Candidate, 100, 5000, 1000, 100, margin,
                "test", new Vector3d(deltaV, 0, 0), burnUT, burnUT + 200, 100);
    }
}
