using MuMech.Landing;
using System;
using System.Threading;
using Xunit;
using Xunit.Abstractions;

namespace MechJebLibTest.LandingPredictionTests
{
    public class TargetAwareAirlessPlannerTests
    {
        private readonly ITestOutputHelper _output;

        public TargetAwareAirlessPlannerTests(ITestOutputHelper output) => _output = output;

        [Fact]
        public void MunSnapshotRefinesSignedDownrangeWithBoundedTerrainQueries()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot();
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => snapshot.TargetTerrainASL, 200, 32, 1024);
            DriveThroughRefinement(planner);

            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.SelectedTerrain.ClearPath);
            Assert.True(planner.SelectedTerrain.HandoffClearance >= 0);
            Assert.InRange(planner.SignedDownrangeError, -200, 200);
            Assert.InRange(planner.CrossrangeError, 100, 200);
            Assert.True(planner.TimingDistanceEstimate < 200);
            Assert.InRange(planner.TerrainQueryCount, 1, 1024);
            _output.WriteLine($"Mun flat-terrain replay: queries={planner.TerrainQueryCount}, " +
                              $"terrainMs={planner.TerrainQueryMilliseconds:F3}, " +
                              $"downrange={planner.SignedDownrangeError:F2}, " +
                              $"crossrange={planner.CrossrangeError:F2}, " +
                              $"timingInterval={planner.TimingInterval:F3}, " +
                              $"transitionVertical={planner.TerminalHandoff.ControllerTransitionVerticalSpeed:F2}, " +
                              $"idealStopDistance={planner.TerminalHandoff.OptimisticVerticalStoppingDistance:F2}");
        }

        [Fact]
        public void V1ControlForecastRefinesMunBrakeReference()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => snapshot.TargetTerrainASL, 200, 32, 1024);
            DriveThroughRefinement(planner);
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.SelectedOutput.UsesV1ControlModel);
            Assert.InRange(planner.SignedDownrangeError, -200, 200);
            _output.WriteLine($"V1 forecast brake={planner.SelectedOutput.BrakeUT:F3} " +
                $"downrange={planner.SignedDownrangeError:F1} " +
                $"crossrange={planner.CrossrangeError:F1} " +
                $"queries={planner.TerrainQueryCount}");
        }

        [Fact]
        public void FailedMunRefresh24FallsBackToARealSafeMissInsteadOfFreezingCorrection()
        {
            // Submission 24 from the 2026-09-29 failed KSP flight. PQS is
            // unavailable offline, so the target-height terrain oracle tests
            // the handoff and search branch rather than actual Mun clearance.
            const double inputUT = 24603579.228602532;
            var source = new AirlessTargetAwareSnapshot(inputUT, inputUT, 200000,
                65138397520.780693, 0.16605670009835341, 138984.37657447575,
                0.67416666666666658, 23.473055555555554, 492.18766502593644,
                692.18766502593644, 8.1128061441219437, 0.2,
                0.019999999552965164, 0.5, -400, 8300,
                new Vector3d(152563.34319155806, 3747.2005581466815, 162815.752504458),
                new Vector3d(-385.34555053115309, 3.243742091594747, 337.63961415603808),
                new Vector3d(0, -4.5207853300062813E-05, 0),
                new Vector3d(-0.079946715521042863, 0, 0.99679913858178948),
                new Vector3d(-0.99679913858178948, 0, -0.079946715521042808),
                new Vector3d(-4.8951627897149581E-18, 1, 6.103432792959659E-17));
            var snapshot = new AirlessTargetAwareSnapshot(source, 78.887671897122345,
                640.0003892624502, 0, 0.27192431688308716, 0,
                200692.18766502594, 1.629016227964847, 8.1128061441219437);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 24, new object(),
                (latitude, longitude) => source.TargetTerrainASL,
                200, 32, 1024);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.ReadyCoarse, planner.Stage);
            planner.BeginCoarseWorker();
            planner.SetCoarseOutputs(planner.RunCoarse());
            DriveUntilSettled(planner);
            if (planner.Stage == TargetAwarePlannerStage.ReadyRefinement)
            {
                planner.BeginRefinementWorker();
                planner.SetRefinement(planner.RunRefinement());
                DriveUntilSettled(planner);
            }
            Assert.True(planner.Stage == TargetAwarePlannerStage.Complete,
                planner.Failure ?? planner.Stage.ToString());
            Assert.True(planner.SelectedTerrain.ClearPath);
            Assert.True(planner.TerminalHandoff.NecessaryControlBoundPasses);
            Assert.True(Math.Abs(planner.SignedDownrangeError) > 200);
            _output.WriteLine($"Captured refresh 24: downrange={planner.SignedDownrangeError:F1} " +
                $"brake={planner.SelectedOutput.BrakeUT:F2} queries={planner.TerrainQueryCount}");
        }

        [Fact]
        public void UnreachableTargetPublishesTerrainClearV1MissForCourseCorrection()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            var farTarget = new AirlessTargetAwareSnapshot(source.InputUT, source.EpochUT,
                source.BodyRadius, source.BodyMu, source.BodyGeeASL,
                source.RotationPeriod, source.TargetLatitude, 90,
                source.TargetTerrainASL, source.DecelEndASL,
                source.MaximumThrustAcceleration, source.Dt, source.MinDt,
                source.MaxOrbits, source.MinimumTerrainASL, source.MaximumTerrainASL,
                source.Position, source.Velocity, source.AngularVelocity,
                source.Axis0, source.Axis90, source.AxisNorth);
            var snapshot = new AirlessTargetAwareSnapshot(farTarget, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => snapshot.TargetTerrainASL, 200, 32, 1024);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.ReadyCoarse, planner.Stage);
            planner.BeginCoarseWorker();
            planner.SetCoarseOutputs(planner.RunCoarse());
            DriveUntilSettled(planner);
            Assert.True(planner.Stage == TargetAwarePlannerStage.Complete,
                planner.Failure ?? planner.Stage.ToString());
            Assert.True(planner.SelectedOutput.UsesV1ControlModel);
            Assert.True(Math.Abs(planner.SignedDownrangeError) > 200);
            Assert.True(planner.SelectedTerrain.ClearPath);
        }

        [Fact]
        public void LiveBrakingForecastRejectsRaisedTerrainWithoutChangingV1Policy()
        {
            var source = AirlessTargetAwareSimulationTests.MunSnapshot();
            Vector3d retrograde = -(source.Velocity -
                Vector3d.Cross(source.AngularVelocity, source.Position)).normalized;
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration,
                true, retrograde);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => 1500, 200, 32, 1024,
                directForecast: true);
            planner.SetDirectOutput(AirlessTargetAwareSimulation.Run(snapshot,
                snapshot.InputUT, true));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.Failed, planner.Stage);
            Assert.Equal("ControllerPathTerrainUnsafe", planner.Failure);
            Assert.Null(planner.SelectedOutput);
        }

        [Fact]
        public void ChangedTerrainDuringRefinementRejectsTheWholeTransaction()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot();
            bool ridgePresent = false;
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => ridgePresent ? 10000 : snapshot.TargetTerrainASL, 200, 32, 1024);
            DriveUntil(planner, TargetAwarePlannerStage.ReadyRefinement);
            ridgePresent = true;
            planner.BeginRefinementWorker();
            planner.SetRefinement(planner.RunRefinement());
            DriveUntilSettled(planner);

            Assert.Equal(TargetAwarePlannerStage.Failed, planner.Stage);
            Assert.Null(planner.SelectedOutput);
        }

        [Fact]
        public void SlowTerrainQueryStopsTheFlightThreadBatchAfterOneSample()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot();
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) =>
                {
                    Thread.Sleep(3);
                    return snapshot.TargetTerrainASL;
                }, 200, 32, 1024, 1);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            planner.AdvanceTerrain();
            Assert.Equal(1, planner.TerrainQueryCount);
            Assert.Equal(TargetAwarePlannerStage.ResolveBallistic, planner.Stage);
        }

        [Fact]
        public void TerrainQueryFailureLeavesNoPublishableCandidate()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot();
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => throw new InvalidOperationException("PQS unavailable"),
                200, 32, 1024);
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(snapshot));
            planner.AdvanceTerrain();
            Assert.Equal(TargetAwarePlannerStage.Failed, planner.Stage);
            Assert.StartsWith("TerrainQueryFailed", planner.Failure);
            Assert.Null(planner.SelectedOutput);
        }

        [Fact]
        public void FarDownrangeRidgeDoesNotHideASeparateClearSignedRoot()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot();
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => longitude > 30 ? 1000 : snapshot.TargetTerrainASL,
                200, 32, 1024);
            DriveThroughRefinement(planner);
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.True(planner.SelectedTerrain.ClearPath);
            Assert.InRange(planner.SignedDownrangeError, -200, 200);
        }

        [Fact]
        public void SelectedCoastIsRecheckedBeforeCommit()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot(16000);
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => snapshot.TargetTerrainASL,
                200, 32, 4096);
            DriveThroughRefinement(planner);
            Assert.Equal(TargetAwarePlannerStage.Complete, planner.Stage);
            Assert.NotEmpty(planner.SelectedOutput.CoastSamples);
        }

        [Fact]
        public void TerrainChangeOnSelectedCoastRejectsTheTransaction()
        {
            var snapshot = AirlessTargetAwareSimulationTests.MunSnapshot(16000);
            bool ridgePresent = false;
            var planner = new TargetAwareAirlessPlanner(snapshot, 1, 1, new object(),
                (latitude, longitude) => ridgePresent ? 30000 : snapshot.TargetTerrainASL,
                200, 32, 4096);
            DriveUntil(planner, TargetAwarePlannerStage.ReadyRefinement);
            planner.BeginRefinementWorker();
            planner.SetRefinement(planner.RunRefinement());
            for (int i = 0; i < 200 && planner.Stage == TargetAwarePlannerStage.ResolveRefinement; ++i)
                planner.AdvanceTerrain();
            Assert.Equal(TargetAwarePlannerStage.ResolveSelectedCoast, planner.Stage);
            ridgePresent = true;
            planner.AdvanceTerrain();
            Assert.Equal(TargetAwarePlannerStage.Failed, planner.Stage);
            Assert.Equal("NoTerrainClearControllerForecast", planner.Failure);
        }

        private void DriveThroughRefinement(TargetAwareAirlessPlanner planner)
        {
            DriveUntil(planner, TargetAwarePlannerStage.ReadyRefinement);
            planner.BeginRefinementWorker();
            planner.SetRefinement(planner.RunRefinement());
            DriveUntilSettled(planner);
            _output.WriteLine($"After refinement: queries={planner.TerrainQueryCount}");
        }

        private void DriveUntil(TargetAwareAirlessPlanner planner, TargetAwarePlannerStage destination)
        {
            planner.SetBallisticSamples(AirlessTargetAwareSimulation.BallisticTerrainPass(planner.Snapshot));
            DriveUntilSettled(planner);
            Assert.Equal(TargetAwarePlannerStage.ReadyCoarse, planner.Stage);
            _output.WriteLine($"After ballistic: queries={planner.TerrainQueryCount}");
            planner.BeginCoarseWorker();
            planner.SetCoarseOutputs(planner.RunCoarse());
            DriveUntilSettled(planner);
            _output.WriteLine($"After coarse: queries={planner.TerrainQueryCount}");
            Assert.True(planner.Stage == destination, planner.Failure ?? planner.Stage.ToString());
        }

        private static void DriveUntilSettled(TargetAwareAirlessPlanner planner)
        {
            for (int i = 0; i < 200 && (planner.Stage == TargetAwarePlannerStage.ResolveBallistic ||
                                          planner.Stage == TargetAwarePlannerStage.ResolveCoarse ||
                                          planner.Stage == TargetAwarePlannerStage.ResolveRefinement ||
                                          planner.Stage == TargetAwarePlannerStage.ResolveDirectForecast ||
                                          planner.Stage == TargetAwarePlannerStage.ResolveSelectedCoast); ++i)
            {
                int before = planner.TerrainQueryCount;
                planner.AdvanceTerrain();
                Assert.InRange(planner.TerrainQueryCount - before, 0, 32);
            }
        }
    }
}
