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
            Assert.Equal("SelectedCoastTerrainIntersection", planner.Failure);
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
                                          planner.Stage == TargetAwarePlannerStage.ResolveSelectedCoast); ++i)
            {
                int before = planner.TerrainQueryCount;
                planner.AdvanceTerrain();
                Assert.InRange(planner.TerrainQueryCount - before, 0, 32);
            }
        }
    }
}
