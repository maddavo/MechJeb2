using MuMech;
using UnityEngine;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class TargetReferenceSnapshotTests
    {
        [Fact]
        public void FuturePlanningSnapshotRetainsOriginalInertialTargetReference()
        {
            var reference = new Vector3d(1234.5, -678.25, 172345.0);
            var source = new LandingGuidanceV2Snapshot(101, 24108733.582233, null,
                new Vector3d(237781.909056, -4568.441665, -23016.381221),
                new Vector3d(50.469649, 5.322262, 519.641017), 36.077968, 810.440636,
                27.717750, 0, 0.165, -130.626389, false, 24108733.582233, reference, true);
            var future = new LandingGuidanceV2Snapshot(source.Version, source.UT + 1800, null,
                source.Position, source.Velocity, source.Mass, source.AvailableDeltaV,
                source.MaximumAcceleration, source.MinimumAcceleration, source.TargetLatitude,
                source.TargetLongitude, false, source.TargetReferenceUT, source.TargetReferencePosition,
                source.HasTargetReferencePosition);

            Assert.True(future.HasTargetReferencePosition);
            Assert.Equal(source.TargetReferenceUT, future.TargetReferenceUT);
            Assert.Equal(reference.x, future.TargetReferencePosition.x);
            Assert.Equal(reference.y, future.TargetReferencePosition.y);
            Assert.Equal(reference.z, future.TargetReferencePosition.z);
            Assert.NotEqual(future.UT, future.TargetReferenceUT);
        }
    }
}
