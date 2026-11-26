using System;
using System.IO;
using MuMech.Landing;
using Xunit;

namespace DunaDynamicsTests
{
    public sealed class DunaM1DiagnosticsRequestTests : IDisposable
    {
        private readonly string directory = Path.Combine(Path.GetTempPath(), "duna-request-" + Guid.NewGuid().ToString("N"));
        private string RequestPath => Path.Combine(directory, DunaM1DiagnosticsRequest.FileName);
        public DunaM1DiagnosticsRequestTests() => Directory.CreateDirectory(directory);
        public void Dispose() => Directory.Delete(directory, true);

        private static string Config(string fields) => "DunaM1Diagnostics\n{\n" + fields + "\n}\n";
        private DunaM1DiagnosticsRequest Load(string fields)
        {
            File.WriteAllText(RequestPath, Config(fields));
            return DunaM1DiagnosticsRequest.Load(RequestPath);
        }

        [Fact]
        public void MissingFileDefaultsToDisabled()
        {
            var r = DunaM1DiagnosticsRequest.Load(RequestPath);
            Assert.False(r.Armed);
            Assert.Equal("missing", r.Status);
            Assert.False(r.TryConsume(out _, out _));
        }
        [Fact]
        public void DisabledRequestIsNotConsumed()
        {
            var r = Load("enabled = false\nforceProbeEnabled = true\nforceProbePhase = DecelerationBurn");
            Assert.False(r.Armed);
            Assert.Equal("disabled", r.Status);
            Assert.False(r.TryConsume(out _, out _));
            Assert.True(File.Exists(RequestPath));
        }
        [Fact]
        public void CaptureOnlyIsExplicitAndForceProbeDefaultsOff()
        {
            var r = Load("enabled = true");
            Assert.True(r.Armed);
            Assert.Null(r.ForcePhase);
        }
        [Fact]
        public void EnabledRequestIsConsumedExactlyOnceAndReceiptPreservesOriginal()
        {
            var r = Load("// calibration\nenabled = true\nforceProbeEnabled = true\nforceProbePhase = DecelerationBurn");
            string original = File.ReadAllText(RequestPath);
            Assert.True(r.Armed);
            Assert.Equal("DecelerationBurn", r.ForcePhase);
            Assert.True(r.TryConsume(out string receipt, out string reason));
            Assert.Equal("consumed", reason);
            Assert.False(r.Armed);
            Assert.False(File.Exists(RequestPath));
            Assert.Equal(original, File.ReadAllText(receipt));
            Assert.StartsWith("DunaM1Diagnostics.consumed-", Path.GetFileName(receipt));
            Assert.False(r.TryConsume(out _, out _));
            Assert.False(DunaM1DiagnosticsRequest.Load(RequestPath).Armed);
        }
        [Theory]
        [InlineData("enabled = 1")]
        [InlineData("forceProbeEnabled = true")]
        [InlineData("enabled = true\nenabled = false")]
        [InlineData("enabled = true\nunknown = true")]
        [InlineData("enabled = true\nforceProbeEnabled = maybe")]
        [InlineData("enabled = true\nforceProbeEnabled = true")]
        [InlineData("enabled = true\nforceProbeEnabled = true\nforceProbePhase = decelerationburn")]
        public void InvalidRequestFailsClosedAndRemainsOnDisk(string fields)
        {
            var r = Load(fields);
            Assert.False(r.Armed);
            Assert.StartsWith("invalid:", r.Status);
            Assert.False(r.TryConsume(out _, out _));
            Assert.True(File.Exists(RequestPath));
        }
        [Fact]
        public void OversizedRequestIsRejected()
        {
            var r = Load("enabled = true\n//" + new string('x', 4096));
            Assert.False(r.Armed);
            Assert.StartsWith("invalid:", r.Status);
        }
        [Fact]
        public void ChangedRequestIsNotConsumed()
        {
            var r = Load("enabled = true");
            File.WriteAllText(RequestPath, Config("enabled = false"));
            Assert.False(r.TryConsume(out _, out string reason));
            Assert.StartsWith("consume_failed:", reason);
            Assert.False(r.Armed);
            Assert.True(File.Exists(RequestPath));
        }
        [Fact]
        public void SecondProcessSnapshotCannotReuseConsumedRequest()
        {
            var a = Load("enabled = true");
            var b = DunaM1DiagnosticsRequest.Load(RequestPath);
            Assert.True(a.TryConsume(out _, out _));
            Assert.False(b.TryConsume(out _, out _));
            Assert.False(b.Armed);
        }
    }
}
