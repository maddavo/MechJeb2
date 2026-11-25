using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text.Json;
using System.Text.RegularExpressions;
using MuMech.Landing;
using Vec = MuMech.Landing.DunaBrakingHandoff.Vec;

internal static class Program
{
    private static double Number(JsonElement e, string key)
    {
        double n = e.GetProperty(key).GetDouble();
        if (!double.IsFinite(n)) throw new InvalidDataException(key + " is nonfinite");
        return n;
    }

    private static Vec Vector(JsonElement e, string key)
    {
        JsonElement a = e.GetProperty(key);
        if (a.GetArrayLength() != 3) throw new InvalidDataException(key + " must contain three components");
        var v = new Vec(a[0].GetDouble(), a[1].GetDouble(), a[2].GetDouble());
        if (!v.Valid) throw new InvalidDataException(key + " is nonfinite");
        return v;
    }

    private static double[] ArrayOf(Vec v) => new[] { v.X, v.Y, v.Z };

    private static string Field(string detail, string key)
    {
        MatchCollection m = Regex.Matches(detail, @"(?:^|\s)" + Regex.Escape(key) + @"=(\S+)");
        if (m.Count != 1) throw new InvalidDataException("Expected exactly one " + key + " in controllerDetail");
        return m[0].Groups[1].Value;
    }

    private static double DetailNumber(string detail, string key)
    {
        double n = double.Parse(Field(detail, key), NumberStyles.Float, CultureInfo.InvariantCulture);
        if (!double.IsFinite(n)) throw new InvalidDataException(key + " is nonfinite");
        return n;
    }

    private static object Describe(DunaBrakingHandoff.Result r) => new
    {
        safe = r.Safe, reason = r.Reason, minClearance_m = r.MinimumClearance,
        endClearance_m = r.EndClearance, endSpeed_m_s = r.EndSpeed, elapsed_s = r.Elapsed
    };

    // Deliberately do not infer body constants or rotation sign here. The caller
    // supplies them explicitly, with provenance in its evidence manifest.
    private static int Main(string[] args)
    {
        try
        {
            if (args.Length != 2) throw new ArgumentException("Usage: DunaProbeReplay <raw-record.json> <body-parameters.json>");
            using JsonDocument capture = JsonDocument.Parse(File.ReadAllText(args[0]));
            using JsonDocument parameters = JsonDocument.Parse(File.ReadAllText(args[1]));
            JsonElement root = capture.RootElement;
            JsonElement record = root.TryGetProperty("record", out JsonElement wrapped) ? wrapped : root;
            JsonElement param = parameters.RootElement;
            JsonElement body = param.TryGetProperty("body", out JsonElement wrappedBody) ? wrappedBody : param;
            string detail = record.GetProperty("controllerDetail").GetString() ?? throw new InvalidDataException("Missing controllerDetail");
            double ut = Number(record, "ut"), probeUT = DetailNumber(detail, "dunaProbeUT");
            if (Math.Abs(ut - probeUT) > 1e-6)
                throw new InvalidDataException("Record is not at the probe epoch; supply the matching state/record, not a later transition.");
            if (record.GetProperty("recordType").GetString() != "guidance_transition" ||
                record.GetProperty("phase").GetString() != "DecelerationBurn" ||
                Field(detail, "dunaBrakeReason") != "delayed_probe_insufficient_hover_authority" ||
                !bool.Parse(Field(detail, "dunaImmediateChecked")))
                throw new InvalidDataException("Not the intended Duna commitment transition.");

            double radius = Number(body, "radius_m"), mu = Number(body, "mu_m3_s2");
            if (radius <= 0 || mu <= 0) throw new InvalidDataException("Body radius and mu must be positive.");
            Vec omega = Vector(body, "angularVelocity_rad_s");
            Vec orbital = Vector(record, "velocity"), surface = Vector(record, "surfaceVelocity");
            Vec up = Vector(record, "up");
            if (Math.Abs(up.Norm - 1) > 1e-6) throw new InvalidDataException("Recorded up is not unit length.");
            double asl = Number(record, "altitudeASL");
            Vec reconstructed = up * ((radius + asl) / up.Norm);
            bool hasPosition = param.TryGetProperty("positionBCI_m", out JsonElement pos) && pos.ValueKind != JsonValueKind.Null;
            Vec position = hasPosition ? Vector(param, "positionBCI_m") : reconstructed;

            double terrain = DetailNumber(detail, "dunaTerrainRadius");
            double authority = DetailNumber(detail, "dunaAuthority");
            double delay = DetailNumber(detail, "dunaProbeDelay");
            double minClearance = DetailNumber(detail, "dunaProbeMinClearance");
            double endClearance = DetailNumber(detail, "dunaProbeEndClearance");
            double endSpeed = DetailNumber(detail, "dunaProbeEndSpeed");
            double immediateMin = DetailNumber(detail, "dunaImmediateMinClearance");
            double elapsed = DetailNumber(detail, "dunaProbeElapsed");
            double gravity = Number(record, "localGravity");

            // These calls are the original implementation, not a reimplementation.
            var delayed = DunaBrakingHandoff.Probe(position, orbital, omega, mu, terrain, authority, delay);
            var immediate = DunaBrakingHandoff.Probe(position, orbital, omega, mu, terrain, authority, 0);
            double gTerrain = mu / (terrain * terrain);
            double gLocal = mu / (position.Norm * position.Norm);
            double bound = gTerrain * Math.Sqrt(1.04);
            double derated = DunaBrakingHandoff.AuthorityFraction * authority;
            double rotationResidual = (orbital - Vec.Cross(omega, position) - surface).Norm;

            var failures = new List<string>();
            // Provisional evidence-comparison tolerances; not flight accuracy limits.
            void Near(string label, double actual, double expected, double tolerance)
            {
                if (!double.IsFinite(actual) || Math.Abs(actual - expected) > tolerance)
                    failures.Add(label + ": " + actual.ToString("R", CultureInfo.InvariantCulture) +
                        " vs " + expected.ToString("R", CultureInfo.InvariantCulture));
            }
            void Check(string label, bool ok) { if (!ok) failures.Add(label); }
            Near("radius/ASL position consistency", (position - reconstructed).Norm, 0, 0.05);
            Near("rotation/surface velocity consistency", rotationResidual, 0, 1e-4);
            Near("local gravity consistency", gLocal, gravity, 1e-5);
            Check("delayed safe flag", delayed.Safe == bool.Parse(Field(detail, "dunaProbeSafe")));
            Check("immediate safe flag", immediate.Safe == bool.Parse(Field(detail, "dunaImmediateSafe")));
            Check("delayed reason", delayed.Reason == Field(detail, "dunaProbeReason"));
            Check("immediate reason", immediate.Reason == Field(detail, "dunaImmediateReason"));
            Check("both rejected before integration", !delayed.Safe && !immediate.Safe &&
                delayed.Reason == "insufficient_hover_authority" && immediate.Reason == delayed.Reason &&
                elapsed == 0 && delayed.Elapsed == 0 && immediate.Elapsed == 0);
            Near("delayed minimum clearance", delayed.MinimumClearance, minClearance, 0.05);
            Near("delayed end clearance", delayed.EndClearance, endClearance, 0.05);
            Near("delayed end speed", delayed.EndSpeed, endSpeed, 1e-4);
            Near("immediate minimum clearance", immediate.MinimumClearance, immediateMin, 0.05);
            Near("immediate end clearance", immediate.EndClearance, immediateMin, 0.05);
            Near("immediate end speed", immediate.EndSpeed, endSpeed, 1e-4);

            Console.WriteLine(JsonSerializer.Serialize(new
            {
                schemaVersion = 1, probeUT, transitionUT = ut,
                positionOrigin = hasPosition ? "explicitly supplied (verify provenance)" : "reconstructed from raw up, ASL and supplied radius",
                positionBCI_m = ArrayOf(position), orbitalVelocity_m_s = ArrayOf(orbital),
                angularVelocity_rad_s = ArrayOf(omega), radius_m = radius, mu_m3_s2 = mu,
                terrainRadius_m = terrain, delay_s = delay, authority_m_s2 = authority,
                deratedAuthority_m_s2 = derated, terrainGravity_m_s2 = gTerrain,
                localGravity_m_s2 = gLocal, hoverBound_m_s2 = bound,
                rawPassesHoverGate = authority > bound, deratedPassesHoverGate = derated > bound,
                rawTerrainTWR = authority / gTerrain, deratedTerrainTWR = derated / gTerrain,
                rotationResidual_m_s = rotationResidual,
                delayed = Describe(delayed), immediate = Describe(immediate),
                pass = failures.Count == 0, failures
            }, new JsonSerializerOptions { WriteIndented = true }));
            return failures.Count == 0 ? 0 : 1;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex.Message);
            return 2;
        }
    }
}
