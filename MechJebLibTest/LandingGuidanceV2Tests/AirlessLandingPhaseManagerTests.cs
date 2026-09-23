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
        public void RecordedMunPlanRequiresWarpExitThenFreshExactIgnitionValidationBeforeThrottle()
        {
            var manager = new AirlessLandingPhaseManager();
            AirlessLandingPlan plan = Candidate(100, 1000, 1200, 30);
            Assert.Equal(AirlessLandingPhaseManagerPhase.WarpToStrategicBurn, manager.Start(plan).Phase);

            AirlessLandingPhaseDecision warp = manager.Tick(900, true, 120, double.NaN);
            Assert.Equal(AirlessLandingPhaseDirective.RequestWarp, warp.Directive);
            Assert.Equal(980, warp.WarpUT, 6);
            Assert.Equal(1, warp.WorkUnits);

            AirlessLandingPhaseDecision exit = manager.Tick(980, true, 120, double.NaN);
            Assert.Equal(AirlessLandingPhaseDirective.ExitWarpAndRequestAttitude, exit.Directive);
            Assert.Equal(AirlessLandingPhaseManagerPhase.AlignStrategicBurn, exit.Phase);

            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, manager.Tick(999.99, true, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequireFreshStrategicValidation, manager.Tick(1000, true, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.Reject, manager.AcceptStrategicValidation(101, 999.99, true).Directive);
        }

        [Fact]
        public void FiniteBurnLeadUsesIgnitionBeforeThePlannedImpulseMidpoint()
        {
            var manager = new AirlessLandingPhaseManager();
            manager.Start(Candidate(100, 1000, 1200, 30), 0.65);
            Assert.Equal(AirlessLandingPhaseDirective.RequestWarp, manager.Tick(900, true, 90, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.ExitWarpAndRequestAttitude, manager.Tick(979.35, true, 90, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, manager.Tick(999.34, false, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequireFreshStrategicValidation, manager.Tick(999.35, false, 0.1, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude,
                manager.AcceptStrategicValidation(101, 999.35, true).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.BeginFiniteBurn, manager.Tick(999.35, false, 0.1, 30).Directive);
        }

        [Fact]
        public void FreshStrategicValidationThenFiniteBurnProgressesToCoast()
        {
            var manager = new AirlessLandingPhaseManager();
            manager.Start(Candidate(100, 1000, 1200, 30));
            manager.Tick(980, true, 90, double.NaN);
            manager.Tick(1000, true, 0.1, double.NaN);
            Assert.Equal(AirlessLandingPhaseDirective.RequestAttitude, manager.AcceptStrategicValidation(101, 1000, true).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.BeginFiniteBurn, manager.Tick(1000, true, 0.1, 30).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequestFiniteBurnThrottle, manager.Tick(1001, true, 0.1, 5).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.FiniteBurnComplete, manager.Tick(1002, true, 0.1, 0.49).Directive);
            Assert.Equal(AirlessLandingPhaseManagerPhase.Coast, manager.Phase);
        }

        [Fact]
        public void PlaneBurnRequiresAChronologicalReplanBeforeStrategicWarp()
        {
            var manager = new AirlessLandingPhaseManager();
            AirlessLandingPlan planePlan = Candidate(100, 1000, 1200, 30, 8, 900);
            Assert.Equal(AirlessLandingPhaseManagerPhase.WarpToPlaneAlignment, manager.Start(planePlan).Phase);
            Assert.Equal(AirlessLandingPhaseDirective.ExitWarpAndRequestAttitude, manager.Tick(880, true, 90, double.NaN).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.BeginFiniteBurn, manager.Tick(900, true, 0.1, 8).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.FiniteBurnComplete, manager.Tick(901, true, 0.1, 0.4).Directive);
            Assert.Equal(AirlessLandingPhaseDirective.RequireStrategicReplan, manager.Tick(902, true, 0.1, 0).Directive);
            AirlessLandingPhaseDecision replan = manager.AdoptStrategicReplan(Candidate(102, 1100, 1300, 28));
            Assert.Equal(AirlessLandingPhaseManagerPhase.WarpToStrategicBurn, replan.Phase);
        }

        [Fact]
        public void SecondPlaneAlignmentReplanFailsClosed()
        {
            var manager = new AirlessLandingPhaseManager();
            manager.Start(Candidate(100, 1000, 1200, 30, 8, 900));
            manager.Tick(880, true, 90, double.NaN);
            manager.Tick(900, true, 0.1, 8);
            manager.Tick(901, true, 0.1, 0.4);
            AirlessLandingPhaseDecision decision = manager.AdoptStrategicReplan(Candidate(102, 1100, 1300, 28, 2, 1050));
            Assert.Equal(AirlessLandingPhaseDirective.Reject, decision.Directive);
            Assert.Contains("another plane burn", decision.Reason);
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
            Assert.Equal(1, decision.WorkUnits);
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