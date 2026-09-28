using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class AirlessTargetAwareSimulationTests
    {
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
    }
}
