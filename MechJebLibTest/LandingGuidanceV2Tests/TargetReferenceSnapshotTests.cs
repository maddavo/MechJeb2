using System.Runtime.Serialization;
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
        [Fact]
        public void TargetPropagationUsesTheSameSurfaceFrameAsKspLongitudeConversion()
        {
            var body = (CelestialBody)FormatterServices.GetUninitializedObject(typeof(CelestialBody));
            body.Radius = 1000;
            body.rotationPeriod = 100;
            // Deliberately opposite to KSP's body-fixed longitude axis. The old
            // V2 code followed this vector and reported a false target corridor.
            body.angularVelocity = Vector3d.up;
            var snapshot = new LandingGuidanceV2Snapshot(102, 1000, body,
                Vector3d.zero, Vector3d.zero, 1, 1, 1, 0, 0, 0, false,
                1000, new Vector3d(1000, 0, 0), true, 0);

            Vector3d atQuarterTurn = AirlessTargetGeometry.SurfacePositionAtUT(snapshot, 1025);
            Vector3d returned = AirlessTargetGeometry.SurfacePositionAtReferenceUT(snapshot, atQuarterTurn, 1025);

            Assert.InRange(atQuarterTurn.x, -1e-9, 1e-9);
            Assert.InRange(atQuarterTurn.y, -1e-9, 1e-9);
            Assert.InRange(atQuarterTurn.z, 999.999999, 1000.000001);
            Assert.InRange(Vector3d.Distance(returned, new Vector3d(1000, 0, 0)), 0, 1e-8);
        }
    }
}
