using System.Collections.Generic;
using MuMech;
using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class TargetAwareTerminalHandoffTests
    {
        [Fact]
        public void HoverKillTravelIsHorizontalAndCannotBeComparedWithVerticalClearance()
        {
            var raw = new AirlessTargetAwareSnapshot(100, 100, 200000,
                65138398000, 0.166, 138984, 0, 23, 492, 692,
                8.1, 0.2, 0.02, 1, -1000, 10000,
                new Vector3d(200700, 0, 0), new Vector3d(-33, 113, 0),
                Vector3d.zero, new Vector3d(1, 0, 0),
                new Vector3d(0, 1, 0), new Vector3d(0, 0, 1));
            var snapshot = new AirlessTargetAwareSnapshot(raw, 78.9, 640, 0,
                0.27, 0, 200692, 1.63, 8.1);
            var endpoint = new AirlessTargetAwareState(new Vector3d(200700, 0, 0),
                new Vector3d(-33, 113, 0), 101);
            var output = new AirlessTargetAwareOutput(100,
                new List<AirlessTargetAwareState>(),
                new List<AirlessTargetAwareState> { endpoint }, true,
                118, 0, 1, true);
            TargetAwareTerminalHandoff handoff =
                TargetAwareTerminalHandoff.Assess(snapshot, output, 492);
            Assert.True(handoff.NecessaryControlBoundPasses);
            Assert.InRange(handoff.RemainingClearance, 207, 209);
            Assert.InRange(handoff.EndHorizontalSpeed, 112, 114);
            Assert.True(handoff.IdealHoverHorizontalStoppingDistance > 19000);
            Assert.True(handoff.IdealHoverHorizontalStoppingTime > 300);
        }
    }
}
