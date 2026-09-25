using MuMech.Landing;
using UnityEngine;
using Xunit;

namespace MechJebLibTest.LandingGuidanceV2Tests
{
    public class TerminalTranslationGuidanceTests
    {
        [Fact]
        public void TiltsAgainstMeasuredHorizontalVelocity()
        {
            Vector3d desired = TerminalTranslationGuidance.DesiredThrustDirection(
                Vector3d.up, Vector3d.right * 4, 0.49);

            Assert.True(Vector3d.Dot(desired, Vector3d.right) < 0);
            Assert.True(Vector3d.Dot(desired, Vector3d.up) > 0);
        }

        [Fact]
        public void CapsLowGravityTiltWhilePreservingVerticalAuthority()
        {
            Vector3d desired = TerminalTranslationGuidance.DesiredThrustDirection(
                Vector3d.up, Vector3d.right * 50, 0.49);

            Assert.True(Vector3d.Dot(desired, Vector3d.up) > 0.80);
            Assert.True(Vector3d.Dot(desired, Vector3d.right) < -0.50);
        }

        [Fact]
        public void UsesVerticalDirectionWhenHorizontalVelocityIsZero()
        {
            Vector3d desired = TerminalTranslationGuidance.DesiredThrustDirection(
                Vector3d.up, Vector3d.up * -2, 0.49);

            Assert.True(Vector3d.Angle(desired, Vector3d.up) < 0.001);
        }
    }
}