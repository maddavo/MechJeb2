using System;
using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class TargetAwareApproachFrameTests
    {
        [Fact]
        public void SignsLongAndShortIndependentlyOfCrossrange()
        {
            Assert.True(TargetAwareApproachFrame.TryCreate(new Vector3d(1, 0, 0),
                new Vector3d(1, 1, 0), 200000, out var frame));

            Assert.True(frame.TryProject(new Vector3d(1, 0.1, 0.2), out double longError,
                out double crossrange));
            Assert.True(frame.TryProject(new Vector3d(1, -0.1, 0.2), out double shortError,
                out double otherCrossrange));
            Assert.True(longError > 0);
            Assert.True(shortError < 0);
            Assert.True(crossrange > 0);
            Assert.Equal(crossrange, otherCrossrange, 8);
        }

        [Fact]
        public void RejectsFrameWithoutAnApproachDirection()
        {
            Assert.False(TargetAwareApproachFrame.TryCreate(new Vector3d(1, 0, 0),
                new Vector3d(2, 0, 0), 200000, out _));
            Assert.False(TargetAwareApproachFrame.TryCreate(new Vector3d(1, 0, 0),
                new Vector3d(1, 1, 0), double.NaN, out _));
        }
    }
}
