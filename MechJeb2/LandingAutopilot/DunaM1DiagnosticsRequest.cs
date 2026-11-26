using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace MuMech.Landing
{
    // Strict, small, KSP-style request file; deliberately no Unity/KSP dependency.
    // Load once at module initialization. Consume by same-directory rename before
    // opening a capture, so a crash cannot leave an enabled request for next launch.
    internal sealed class DunaM1DiagnosticsRequest
    {
        internal const string FileName = "DunaM1Diagnostics.cfg";
        private const int MaxBytes = 4096;
        private static readonly HashSet<string> Phases = new HashSet<string>(StringComparer.Ordinal)
        {
            "DeorbitBurn", "PlaneChange", "LowDeorbitBurn", "CourseCorrection",
            "CoastToDeceleration", "DecelerationBurn", "KillHorizontalVelocity", "FinalDescent"
        };
        private readonly string path, snapshot;
        internal bool Armed { get; private set; }
        internal string ForcePhase { get; }
        internal string Status { get; private set; }
        private DunaM1DiagnosticsRequest(string path, string snapshot, bool armed, string phase, string status)
        {
            this.path = path; this.snapshot = snapshot; Armed = armed; ForcePhase = phase; Status = status;
        }
        private static string ReadBounded(string path)
        {
            using (var file = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read))
            {
                if (file.Length > MaxBytes) throw new InvalidDataException("request exceeds 4096 bytes");
                using (var reader = new StreamReader(file, new UTF8Encoding(false, true), true))
                {
                    var buffer = new char[MaxBytes + 1];
                    int n = reader.ReadBlock(buffer, 0, buffer.Length);
                    if (n > MaxBytes) throw new InvalidDataException("request exceeds 4096 characters");
                    return new string(buffer, 0, n);
                }
            }
        }
        internal static DunaM1DiagnosticsRequest Load(string path)
        {
            try
            {
                string raw = ReadBounded(path);
                var lines = new List<string>();
                using (var reader = new StringReader(raw))
                {
                    string line;
                    while ((line = reader.ReadLine()) != null)
                    {
                        int comment = line.IndexOf("//", StringComparison.Ordinal);
                        if (comment >= 0) line = line.Substring(0, comment);
                        line = line.Trim();
                        if (line.Length != 0) lines.Add(line);
                    }
                }
                if (lines.Count < 4 || lines[0] != "DunaM1Diagnostics" || lines[1] != "{" ||
                    lines[lines.Count - 1] != "}") throw new InvalidDataException("expected one DunaM1Diagnostics block");
                var fields = new Dictionary<string, string>(StringComparer.Ordinal);
                for (int i = 2; i < lines.Count - 1; ++i)
                {
                    int eq = lines[i].IndexOf('=');
                    if (eq < 1) throw new InvalidDataException("expected key = value");
                    string key = lines[i].Substring(0, eq).Trim(), value = lines[i].Substring(eq + 1).Trim();
                    if (key != "enabled" && key != "forceProbeEnabled" && key != "forceProbePhase")
                        throw new InvalidDataException("unknown key: " + key);
                    if (fields.ContainsKey(key)) throw new InvalidDataException("duplicate key: " + key);
                    fields.Add(key, value);
                }
                if (!fields.TryGetValue("enabled", out string enabledText) || !bool.TryParse(enabledText, out bool enabled))
                    throw new InvalidDataException("enabled must be true or false");
                bool force = false;
                if (fields.TryGetValue("forceProbeEnabled", out string forceText) && !bool.TryParse(forceText, out force))
                    throw new InvalidDataException("forceProbeEnabled must be true or false");
                fields.TryGetValue("forceProbePhase", out string phase);
                if ((force && string.IsNullOrEmpty(phase)) || (phase != null && !Phases.Contains(phase)))
                    throw new InvalidDataException("forceProbePhase must name a supported targeted landing phase");
                return new DunaM1DiagnosticsRequest(path, raw, enabled, force ? phase : null, enabled ? "armed" : "disabled");
            }
            catch (FileNotFoundException) { return new DunaM1DiagnosticsRequest(path, null, false, null, "missing"); }
            catch (DirectoryNotFoundException) { return new DunaM1DiagnosticsRequest(path, null, false, null, "missing"); }
            catch (Exception ex) { return new DunaM1DiagnosticsRequest(path, null, false, null, "invalid: " + ex.Message); }
        }
        internal bool TryConsume(out string receipt, out string reason)
        {
            receipt = null;
            if (!Armed) { reason = Status; return false; }
            Armed = false;
            try
            {
                // Preserve the exact original request as a receipt. Never overwrite
                // an existing receipt or an independently created request.
                string destination = Path.Combine(Path.GetDirectoryName(path) ?? "",
                    "DunaM1Diagnostics.consumed-" + Guid.NewGuid().ToString("N") + ".cfg");
                if (!string.Equals(ReadBounded(path), snapshot, StringComparison.Ordinal))
                    throw new InvalidDataException("request changed since initialization; restart KSP to rearm");
                File.Move(path, destination);
                receipt = destination;
                if (!string.Equals(ReadBounded(destination), snapshot, StringComparison.Ordinal))
                    throw new InvalidDataException("consumed request changed; no capture opened");
                Status = reason = "consumed";
                return true;
            }
            catch (Exception ex) { Status = reason = "consume_failed: " + ex.Message; return false; }
        }
    }
}
