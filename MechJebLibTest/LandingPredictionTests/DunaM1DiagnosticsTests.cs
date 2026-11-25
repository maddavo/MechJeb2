using System;
using MuMech;
using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class DunaM1DiagnosticsTests
    {
        private static AbsoluteVector At(double ut) => new AbsoluteVector { UT = ut, Radius = 320000 };

        [Fact]
        public void StartIsNotFirstImpulseAndAcceptedIntervalIsNotNextInterval()
        {
            var d = new DunaM1Prediction("test", 1, 9);
            d.Start(100);
            d.ObserveStep(At(100.5), .5, 1, 0, 5, 100, -10, 1, 0);
            Assert.True(double.IsNaN(d.FirstVirtualBrakeUT));
            d.ObserveStep(At(101), .5, 1, 5, 5, 95, -10, 1, 0);
            Assert.Equal(100, d.IntegrationStartUT);
            Assert.Equal(101, d.FirstVirtualBrakeUT);
            Assert.Equal(.5, d.FirstBrakeAcceptedDt);
            Assert.Equal(1, d.FirstBrakeLimiterDt);
            Assert.Equal(1, d.ExcessSteps);
            Assert.Equal(2.5, d.ExcessDv);
            Assert.Equal(5, d.VirtualDv);
            Assert.Equal(2, d.Samples.Count); // First impulse is retained even between periodic samples.
        }

        [Fact]
        public void SamplingBoundDoesNotStopAccounting()
        {
            var d = new DunaM1Prediction("test", 2, 9);
            d.Start(0);
            for (int i = 1; i <= 300; i++)
                d.ObserveStep(At(5*i), 5, 5, i, 5, 10, -1, 0, 0);
            Assert.Equal(DunaM1Prediction.MaxSamples, d.Samples.Count);
            Assert.True(d.SamplesTruncated);
            Assert.Equal(300, d.AcceptedSteps);
            Assert.Equal(300, d.BrakingSteps);
            Assert.Equal(300, d.VirtualDv);
            Assert.Equal(0, d.ExcessSteps);
        }

        [Fact]
        public void OrdinaryGeometricContactNeverClaimsSafeLanding()
        {
            Assert.Equal("geometric_contact_only", DunaM1Prediction.ContactMeaning(ReentrySimulation.Outcome.LANDED));
            Assert.Equal("impact", DunaM1Prediction.ContactMeaning(ReentrySimulation.Outcome.IMPACT));
            Assert.Equal("no_geometric_contact", DunaM1Prediction.ContactMeaning(ReentrySimulation.Outcome.NO_REENTRY));
            Assert.Equal("null", DunaM1Capture.N(double.NaN));
            Assert.Equal("\"a\\u000ab\\\\\\\"\"", DunaM1Capture.Q("a\nb\\\""));
        }

        [Fact]
        public void ObserverDoesNotModifyPassedStateOrImpulse()
        {
            var d = new DunaM1Prediction("test", 3, 9);
            var pos = At(42);
            double accepted = .02, next = .5, impulse = 2.5;
            d.ObserveStep(pos, accepted, next, impulse, 5, 100, -10, 1, 0);
            Assert.Equal(42, pos.UT);
            Assert.Equal(.02, accepted);
            Assert.Equal(.5, next);
            Assert.Equal(2.5, impulse);
            Assert.Equal(2.4, d.ExcessDv, 10);
        }
    }
}
