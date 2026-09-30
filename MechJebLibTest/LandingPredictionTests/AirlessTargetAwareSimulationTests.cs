using MuMech.Landing;
using System;
using Xunit;
using Xunit.Abstractions;

namespace MechJebLibTest.LandingPredictionTests
{
    public class AirlessTargetAwareSimulationTests
    {
        private readonly ITestOutputHelper _output;
        public AirlessTargetAwareSimulationTests(ITestOutputHelper output) => _output = output;
        // Captured Mun submission 177, session a398605cbd7942b0837d4fe4a3493229.
        // The fixture is immutable numerical evidence from the installed passive logger.
        internal static AirlessTargetAwareSnapshot MunSnapshot(double maximumTerrainASL = 10000) =>
            new AirlessTargetAwareSnapshot(
            24603456.768967465, 24603456.808967464, 200000, 65138397520.78069,
            0.1660567000983534, 138984.37657447223, 0.6741666666666666,
            23.473055555555554, 492.18766502593644, 692.1876650259364,
            8.059969754789321, 0.2, 0.019999999552965164, 0.5,
            -10000, maximumTerrainASL,
            new Vector3d(169119.97759942123, 3204.0130791774627, 147215.54897786683),
            new Vector3d(-344.2340528178373, 5.522923460620535, 393.87087909277227),
            new Vector3d(0, -4.520785330006396e-05, 0),
            new Vector3d(-0.24958996806572017, 0, 0.9683516137441776),
            new Vector3d(-0.9683516137441776, 0, -0.24958996806572012),
            new Vector3d(-1.5282473037180257e-17, 1, 5.929247694626362e-17));

        [Fact]
        public void TerminalTerrainContactUsesVesselBottomOffset()
        {
            var source = MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration,
                bottomOffset: 3);
            var path = new System.Collections.Generic.List<AirlessTargetAwareState>
            {
                new AirlessTargetAwareState(
                    new Vector3d(source.BodyRadius + 10, 0, 0), Vector3d.zero,
                    source.InputUT),
                new AirlessTargetAwareState(
                    new Vector3d(source.BodyRadius + 2, 0, 0), Vector3d.zero,
                    source.InputUT + 1)
            };
            var offline = TargetAwareTerminalTerrainProbe.Evaluate(snapshot, path,
                (latitude, longitude) => 0, 64);
            var incremental = new TargetAwareTerminalTerrainResolver(snapshot, path,
                (latitude, longitude) => 0, 64);
            while (!incremental.Done)
                incremental.Advance(4, 1000);
            Assert.True(offline.Resolved && offline.HasContact);
            Assert.True(incremental.Result.Resolved && incremental.Result.HasContact);
            Assert.InRange(offline.Contact.Position.magnitude - source.BodyRadius, 2.9, 3.1);
            Assert.InRange(incremental.Result.Contact.Position.magnitude - source.BodyRadius,
                2.9, 3.1);
        }

        [Fact]
        public void ReplaysCapturedMunCandidate177UsingOneImmutableSnapshot()
        {
            var snapshot = MunSnapshot();
            var output = AirlessTargetAwareSimulation.Run(snapshot, 24603984.289533857);
            Assert.True(output.ReachedHandoff);
            var endpoint = AirlessTargetAwareSimulation.ToAbsolute(output.End.Position, output.End.UT, snapshot);
            Assert.InRange(endpoint.Latitude, 0.631151244, 0.631151245);
            Assert.InRange(endpoint.Longitude, 23.18660344, 23.18660346);
            Assert.InRange(endpoint.Radius, 200491.964, 200491.966);
            Assert.InRange(endpoint.UT, 24604187.4944, 24604187.4947);
            Assert.InRange(output.EndSurfaceSpeed, 71.846, 71.848);
            Assert.Equal(149, output.Steps);
            Assert.InRange(output.VirtualDeltaV, 564.393, 564.395);
        }

        [Fact]
        public void Candidate177HasValidClearanceButInsufficientIdealVerticalStopDistance()
        {
            var snapshot = MunSnapshot();
            var output = AirlessTargetAwareSimulation.Run(snapshot, 24603984.289533857);
            var terminal = TargetAwareTerminalHandoff.Assess(snapshot, output, 450.18037516254117);
            Assert.InRange(terminal.RemainingClearance, 41, 43);
            Assert.InRange(terminal.ControllerTransitionVerticalSpeed, -54, -53);
            Assert.True(terminal.OptimisticVerticalStoppingDistance >
                        terminal.ControllerTransitionClearance);
            Assert.False(terminal.NecessaryControlBoundPasses);
        }

        [Fact]
        public void V1ForecastUsesFiveSecondGuardAndActualThrottleLaw()
        {
            var source = MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79.0, 640.0, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var output = AirlessTargetAwareSimulation.Run(snapshot, 24603984.289533857, true);
            var endpoint = AirlessTargetAwareSimulation.ToAbsolute(output.End.Position,
                output.End.UT, snapshot);
            _output.WriteLine($"V1 brake reference={output.BrakeUT:F2} burnStart={output.Trajectory[0].UT:F2} " +
                $"endUT={output.End.UT:F2} latitude={endpoint.Latitude:F4} longitude={endpoint.Longitude:F4} " +
                $"endASL={endpoint.Radius - snapshot.BodyRadius:F1} surfaceSpeed={output.EndSurfaceSpeed:F1} " +
                $"reached={output.ReachedHandoff}");
            Vector3d surface = output.End.Velocity -
                Vector3d.Cross(snapshot.AngularVelocity, output.End.Position);
            Vector3d up = output.End.Position.normalized;
            _output.WriteLine($"handoff vertical={Vector3d.Dot(surface, up):F2} " +
                $"horizontal={Vector3d.Exclude(up, surface).magnitude:F2}");
            var horizontalKill = AirlessTargetAwareSimulation.RunIdealHorizontalKillBound(
                snapshot, output.End, output.EndMass);
            Assert.True(horizontalKill.ReachedFinalDescent);
            Assert.InRange(horizontalKill.EndHorizontalSpeed, 0, 1);
            _output.WriteLine($"ideal KHV endH={horizontalKill.EndHorizontalSpeed:F2} " +
                $"time={horizontalKill.Trajectory[horizontalKill.Trajectory.Count - 1].UT - output.End.UT:F1} " +
                $"distance={(horizontalKill.Trajectory[horizontalKill.Trajectory.Count - 1].Position - output.End.Position).magnitude:F1}");
            foreach (double bottomOffset in new[] { 0.0, 3.0, 10.0 })
            {
                var final = AirlessTargetAwareSimulation.RunIdealFinalDescentBound(
                    snapshot, horizontalKill, source.TargetTerrainASL, 0.5, 0,
                    bottomOffset);
                _output.WriteLine($"ideal final bottomOffset={bottomOffset:F1} " +
                    $"reached={final.ReachedTerrain} vertical={final.ContactVerticalSpeed:F2} " +
                    $"horizontal={final.ContactHorizontalSpeed:F2} " +
                    $"time={final.Trajectory[final.Trajectory.Count - 1].UT - horizontalKill.Trajectory[horizontalKill.Trajectory.Count - 1].UT:F1}");
                Assert.True(final.ReachedTerrain);
                Assert.InRange(Math.Abs(final.ContactVerticalSpeed), 0, 0.5);
                Assert.InRange(final.ContactHorizontalSpeed, 0, 1);
            }
            var terminal = AirlessTargetAwareSimulation.RunNominalTerminal(
                snapshot, output, source.TargetTerrainASL);
            var terminalStart = AirlessTargetAwareSimulation.ToAbsolute(
                terminal.Trajectory[0].Position, terminal.Trajectory[0].UT, snapshot);
            var terminalEnd = AirlessTargetAwareSimulation.ToAbsolute(
                terminal.End.Position, terminal.End.UT, snapshot);
            _output.WriteLine($"terminal longitude={terminalStart.Longitude:F6}..{terminalEnd.Longitude:F6}");
            Assert.True(terminal.HorizontalKill.ReachedFinalDescent);
            Assert.True(terminal.FinalDescent.ReachedTerrain);
            Assert.True(terminal.End.UT > output.End.UT);
            var terrain = TargetAwareTerminalTerrainProbe.Evaluate(snapshot,
                terminal.Trajectory, (latitude, longitude) => source.TargetTerrainASL, 1024);
            Assert.True(terrain.Resolved);
            Assert.True(terrain.HasContact);
            Assert.InRange(Math.Abs(terrain.Contact.UT - terminal.End.UT), 0, 0.25);
            var incremental = new TargetAwareTerminalTerrainResolver(snapshot,
                terminal.Trajectory, (latitude, longitude) => source.TargetTerrainASL, 1024);
            int terrainTicks = 0;
            while (!incremental.Done && terrainTicks++ < 1024)
                incremental.Advance(4, 1000);
            Assert.True(incremental.Done);
            Assert.True(incremental.Result.Resolved);
            Assert.True(incremental.Result.HasContact);
            Assert.True(terrainTicks > 1);
            Assert.Equal(terrain.QueryCount, incremental.QueryCount);
            Assert.InRange(Math.Abs(incremental.Result.Contact.UT - terrain.Contact.UT), 0, 1e-7);
            var exhausted = new TargetAwareTerminalTerrainResolver(snapshot,
                terminal.Trajectory, (latitude, longitude) => source.TargetTerrainASL, 1);
            while (!exhausted.Done)
                exhausted.Advance(1, 1000);
            Assert.False(exhausted.Result.Resolved);
            Assert.False(exhausted.Result.HasContact);
            var unavailable = new TargetAwareTerminalTerrainResolver(snapshot,
                terminal.Trajectory, (latitude, longitude) =>
                    throw new InvalidOperationException("PQS unavailable"), 1024);
            unavailable.Advance(4, 1000);
            Assert.True(unavailable.Done);
            Assert.False(unavailable.Result.Resolved);
            Assert.Equal(1, unavailable.QueryCount);
            var insufficientBudget = TargetAwareTerminalTerrainProbe.Evaluate(snapshot,
                terminal.Trajectory, (latitude, longitude) => source.TargetTerrainASL, 1);
            Assert.False(insufficientBudget.Resolved);
            Assert.False(insufficientBudget.HasContact);
            var risingTerrain = TargetAwareTerminalTerrainProbe.Evaluate(snapshot,
                terminal.Trajectory,
                (latitude, longitude) => longitude >= 23.1695 ? 1000 :
                    source.TargetTerrainASL, 1024);
            Assert.True(risingTerrain.Resolved);
            Assert.True(risingTerrain.HasContact);
            Assert.True(risingTerrain.Contact.UT < terrain.Contact.UT);
            var incrementalRidge = new TargetAwareTerminalTerrainResolver(snapshot,
                terminal.Trajectory,
                (latitude, longitude) => longitude >= 23.1695 ? 1000 :
                    source.TargetTerrainASL, 1024);
            while (!incrementalRidge.Done)
                incrementalRidge.Advance(4, 1000);
            Assert.True(incrementalRidge.Result.Resolved);
            Assert.True(incrementalRidge.Result.HasContact);
            Assert.Equal(risingTerrain.QueryCount, incrementalRidge.QueryCount);
            Assert.InRange(Math.Abs(incrementalRidge.Result.Contact.UT -
                risingTerrain.Contact.UT), 0, 1e-7);
            Assert.True(output.UsesV1ControlModel);
            Assert.InRange(output.Trajectory[0].UT, 24603979.28, 24603979.30);
            Assert.True(output.VirtualDeltaV > 0);
            Assert.True(output.Steps > 0);
            // Independent Python replay of the same captured Mun state.
            Assert.InRange(endpoint.Latitude, 0.63140, 0.63143);
            Assert.InRange(endpoint.Longitude, 23.1676, 23.1678);
            Assert.InRange(output.End.UT, 24604188.53, 24604188.63);
            Assert.InRange(output.EndSurfaceSpeed, 8.6, 8.8);
        }

        [Fact]
        public void TerminalReplayDoesNotClaimTouchdownFromUnmodelledAbove300MeterBranch()
        {
            var source = MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var handoff = new AirlessTargetAwareState(
                new Vector3d(source.BodyRadius + source.TargetTerrainASL + 800, 0, 0),
                new Vector3d(-5, 0, 0), source.InputUT);
            var horizontalKill = AirlessTargetAwareSimulation.RunIdealHorizontalKillBound(
                snapshot, handoff, snapshot.InitialMass);
            Assert.True(horizontalKill.ReachedFinalDescent);
            var final = AirlessTargetAwareSimulation.RunIdealFinalDescentBound(
                snapshot, horizontalKill, source.TargetTerrainASL, 0.5, 0);
            Assert.False(final.ReachedTerrain);
        }

        [Fact]
        public void LastCapturedMunBrakingStateExposesTheRemainingHorizontalKillBurden()
        {
            // Last complete braking input from session
            // 10ee86d75bca457eb20073c71cbd52b5. The installed DLL used
            // 1807.553 m ASL for its handoff. Replay the approved current
            // target-terrain + 200 m V1 policy to measure the consequence,
            // not to claim this is an observed flight under the new source.
            const double inputUT = 24604103.74829403;
            const double targetTerrain = 492.18766502593644;
            var raw = new AirlessTargetAwareSnapshot(inputUT, inputUT, 200000,
                65138397520.78069, 0.1660567000983534, 138984.37657447546,
                0.6741666666666666, 23.473055555555554, targetTerrain,
                targetTerrain + 200, 9.749190150489483, 0.2,
                0.019999999552965164, 0.5, -400, 8300,
                new Vector3d(-120949.30274985329, 2378.290617805737,
                    161615.9700111877),
                new Vector3d(-91.16591588771323, -2.133691542577489,
                    -112.53648454495256),
                new Vector3d(0, -4.52078533000629e-05, 0),
                new Vector3d(-0.231783882227675, 0, 0.9727673061629216),
                new Vector3d(-0.9727673061629216, 0, -0.23178388222767493),
                new Vector3d(-1.419220074448142e-17, 1,
                    5.956285119588958e-17));
            var snapshot = new AirlessTargetAwareSnapshot(raw,
                65.6465048286409, 640.0002582894462, 0,
                0.27192431688308716, 0, 200000 + targetTerrain + 200,
                1.629016227964847, 9.749190150489483, true,
                new Vector3d(0.6163865923881531, 0.015680503100156784,
                    0.7872875928878784), 0, 1, 0, 1, 0.5);
            var braking = AirlessTargetAwareSimulation.RunNominalV1(snapshot);
            Assert.True(braking.ReachedHandoff);
            Vector3d surface = braking.End.Velocity -
                Vector3d.Cross(snapshot.AngularVelocity, braking.End.Position);
            double horizontal = Vector3d.Exclude(braking.End.Position.normalized,
                surface).magnitude;
            double vertical = Vector3d.Dot(surface, braking.End.Position.normalized);
            var kill = AirlessTargetAwareSimulation.RunIdealHorizontalKillBound(
                snapshot, braking.End, braking.EndMass);
            var delayedKill = AirlessTargetAwareSimulation.RunDelayedHorizontalKillBound(
                snapshot, braking.End, braking.EndMass, 5.1);
            Assert.True(delayedKill.ReachedFinalDescent);
            var delayedFinal = AirlessTargetAwareSimulation.RunIdealFinalDescentBound(
                snapshot, delayedKill, targetTerrain, 0.5, 0);
            Assert.True(delayedFinal.ReachedTerrain);
            var delayedTerrain = TargetAwareTerminalTerrainProbe.Evaluate(snapshot,
                delayedKill.Trajectory, (latitude, longitude) => targetTerrain, 1024);
            Assert.True(delayedTerrain.Resolved);
            Assert.False(delayedTerrain.HasContact);
            var terminal = AirlessTargetAwareSimulation.RunNominalTerminal(
                snapshot, braking, targetTerrain);
            var terrain = TargetAwareTerminalTerrainProbe.Evaluate(snapshot,
                terminal.Trajectory, (latitude, longitude) => targetTerrain, 1024);
            Assert.True(terminal.HorizontalKill.ReachedFinalDescent);
            Assert.True(terminal.FinalDescent.ReachedTerrain);
            Assert.True(terrain.Resolved);
            Assert.True(terrain.HasContact);
            double terminalEndpointSeparation =
                (delayedFinal.Trajectory[delayedFinal.Trajectory.Count - 1].Position -
                    terminal.End.Position).magnitude;
            _output.WriteLine($"captured-input/current-policy handoff UT={braking.End.UT:F2} " +
                $"ASL={braking.End.Position.magnitude - snapshot.BodyRadius:F1} " +
                $"horizontal={horizontal:F2} vertical={vertical:F2} idealKillTime=" +
                $"{kill.Trajectory[kill.Trajectory.Count - 1].UT - braking.End.UT:F1} " +
                $"idealKillDistance=" +
                $"{(kill.Trajectory[kill.Trajectory.Count - 1].Position - braking.End.Position).magnitude:F1} " +
                $"terminalVertical={terminal.FinalDescent.ContactVerticalSpeed:F2} " +
                $"terminalHorizontal={terminal.FinalDescent.ContactHorizontalSpeed:F2} " +
                $"delayedVertical={delayedFinal.ContactVerticalSpeed:F2} " +
                $"delayedHorizontal={delayedFinal.ContactHorizontalSpeed:F2} " +
                $"terminalEndpointSeparation={terminalEndpointSeparation:F1}");
        }

        [Fact]
        public void NominalBrakeReferenceIsIndependentOfTheSelectedTarget()
        {
            var source = MunSnapshot();
            var alternateTarget = new AirlessTargetAwareSnapshot(source.InputUT,
                source.EpochUT, source.BodyRadius, source.BodyMu, source.BodyGeeASL,
                source.RotationPeriod, -15, -80, source.TargetTerrainASL,
                source.DecelEndASL, source.MaximumThrustAcceleration, source.Dt,
                source.MinDt, source.MaxOrbits, source.MinimumTerrainASL,
                source.MaximumTerrainASL, source.Position, source.Velocity,
                source.AngularVelocity, source.Axis0, source.Axis90, source.AxisNorth);
            var first = new AirlessTargetAwareSnapshot(source, 79, 640, 0, 0.27, 0,
                source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var second = new AirlessTargetAwareSnapshot(alternateTarget, 79, 640, 0, 0.27, 0,
                source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            var resultA = AirlessTargetAwareSimulation.RunNominalV1(first);
            var resultB = AirlessTargetAwareSimulation.RunNominalV1(second);
            Assert.Equal(resultA.BrakeUT, resultB.BrakeUT);
            Assert.Equal(resultA.End.Position.x, resultB.End.Position.x);
            Assert.Equal(resultA.End.Position.y, resultB.End.Position.y);
            Assert.Equal(resultA.End.Position.z, resultB.End.Position.z);
        }

        [Fact]
        public void MunV1ControllerBrakeWindowHasReachableTarget()
        {
            var source = MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79.0, 640.0, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration);
            double first = double.NaN, last = double.NaN;
            for (double brake = 24603960; brake <= 24604080; brake += 20)
            {
                var output = AirlessTargetAwareSimulation.Run(snapshot, brake, true);
                var endpoint = AirlessTargetAwareSimulation.ToAbsolute(output.End.Position,
                    output.End.UT, snapshot);
                _output.WriteLine($"brake={brake:F1} reached={output.ReachedHandoff} " +
                    $"longitude={endpoint.Longitude:F4} speed={output.EndSurfaceSpeed:F1}");
                if (double.IsNaN(first)) first = endpoint.Longitude;
                last = endpoint.Longitude;
            }
            Assert.True(first < snapshot.TargetLongitude && last > snapshot.TargetLongitude);
        }

        [Fact]
        public void LiveForecastRejectsUnresolvedAttitudeInsteadOfInventingImmediateThrust()
        {
            var source = MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration,
                true, source.Velocity.normalized);
            Assert.Throws<InvalidOperationException>(() =>
                AirlessTargetAwareSimulation.Run(snapshot, snapshot.InputUT, true));
        }

        [Fact]
        public void LiveForecastUsesTheSameSeventyFivePercentAttitudeGateAsV1()
        {
            var source = MunSnapshot();
            Vector3d retrograde = -(source.Velocity -
                Vector3d.Cross(source.AngularVelocity, source.Position)).normalized;
            Vector3d sideways = Vector3d.Cross(retrograde, source.Position.normalized).normalized;
            Vector3d forward = (0.80 * retrograde + 0.60 * sideways).normalized;
            var snapshot = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration,
                true, forward);
            var result = AirlessTargetAwareSimulation.Run(snapshot, snapshot.InputUT, true);
            Assert.True(result.UsesV1ControlModel);
            var aligned = new AirlessTargetAwareSnapshot(source, 79, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, source.MaximumThrustAcceleration,
                true, retrograde);
            var alignedResult = AirlessTargetAwareSimulation.Run(aligned,
                aligned.InputUT, true);
            Assert.True((result.Trajectory[1].Velocity -
                         alignedResult.Trajectory[1].Velocity).magnitude > 0.001);
        }

        [Fact]
        public void MunHorizontalKillIdealBoundNeedsKilometresAtHighLateralSpeed()
        {
            var source = MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 70, 640, 0,
                0, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, 640.0 / 70);
            var handoff = new AirlessTargetAwareState(new Vector3d(0, 0, 200700),
                new Vector3d(100, 0, 0), source.InputUT);
            var bound = AirlessTargetAwareSimulation.RunIdealHorizontalKillBound(
                snapshot, handoff, 70);
            _output.WriteLine($"ideal KHV time={bound.Trajectory[bound.Trajectory.Count - 1].UT - handoff.UT:F1} " +
                $"travel={(bound.Trajectory[bound.Trajectory.Count - 1].Position - handoff.Position).magnitude:F1} " +
                $"endH={bound.EndHorizontalSpeed:F2}");
            Assert.True(bound.ReachedFinalDescent);
            Assert.InRange(bound.EndHorizontalSpeed, 0, 1);
            Assert.True(bound.Trajectory[bound.Trajectory.Count - 1].UT - handoff.UT > 200);
            Assert.True((bound.Trajectory[bound.Trajectory.Count - 1].Position -
                         handoff.Position).magnitude > 10000);
            var terrain = TargetAwareTerminalTerrainProbe.Evaluate(snapshot,
                bound.Trajectory, (latitude, longitude) => 0, 1024);
            Assert.True(terrain.Resolved);
            Assert.False(terrain.HasContact);
            Assert.True(terrain.QueryCount <= 1024);
        }

        [Theory]
        [InlineData(-5)]
        [InlineData(-15)]
        [InlineData(-25)]
        public void ApprovedFinalDescentEnvelopeMeetsRequestedTouchdownSpeedInIdealMunReplay(
            double startingVerticalSpeed)
        {
            var source = MunSnapshot();
            var snapshot = new AirlessTargetAwareSnapshot(source, 70, 640, 0,
                0.27, 0, source.BodyRadius + source.DecelEndASL,
                source.BodyGeeASL * 9.81, 640.0 / 70);
            double radius = source.BodyRadius + source.TargetTerrainASL + 200;
            var position = new Vector3d(0, 0, radius);
            var state = new AirlessTargetAwareState(position,
                Vector3d.Cross(source.AngularVelocity, position) +
                startingVerticalSpeed * position.normalized, source.InputUT);
            var handoff = new AirlessHorizontalKillBound(
                new System.Collections.Generic.List<AirlessTargetAwareState> { state },
                true, 0, 0, 70);
            var final = AirlessTargetAwareSimulation.RunIdealFinalDescentBound(
                snapshot, handoff, source.TargetTerrainASL, 0.5, 0);
            _output.WriteLine($"initialV={startingVerticalSpeed:F0} contactV={final.ContactVerticalSpeed:F2} " +
                $"time={final.Trajectory[final.Trajectory.Count - 1].UT - state.UT:F1}");
            Assert.True(final.ReachedTerrain);
            Assert.InRange(Math.Abs(final.ContactVerticalSpeed), 0, 0.5);
        }
    }
}
