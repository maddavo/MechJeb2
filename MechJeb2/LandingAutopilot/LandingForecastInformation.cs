using System;
using System.Collections.Generic;

namespace MuMech.Landing
{
    // Read-only accounting/presentation. Never feeds a control law.
    internal static class LandingForecastInformation
    {
        internal const int PredictionRows = 10;
        internal const int CorrectionRows = 4;
        internal const float RowHeight = 20;

        internal static string DeltaV(double value) =>
            !double.IsNaN(value) && !double.IsInfinity(value) && value >= 0
                ? value.ToString(value < 10000 ? "F1" : "G4") + " m/s" : "unavailable";

        internal static string SingleLine(string value) => (value ?? "").Replace("\r", " ").Replace("\n", " ");

        internal static double DeliveryImpulse(double acceleration, double mass, double massFlow, double seconds)
        {
            if (seconds <= 0 || acceleration == 0) return 0;
            if (mass <= 0 || massFlow < 0 || seconds > .5 || double.IsNaN(acceleration)) return double.NaN;
            return massFlow == 0 ? acceleration * seconds : acceleration * mass / massFlow *
                Math.Log(mass / Math.Max(.01 * mass, mass - massFlow * seconds));
        }

        internal static string[] PredictionLines(string state, bool present, bool impact, string coordinates,
            string altitude, string error, double deltaV, double age, string time, string biome = "—", double maxDrag = 0, double surfaceSpeed = double.NaN)
        {
            var rows = new string[PredictionRows];
            rows[0] = state;
            if (present)
            {
                rows[1] = impact ? "Terrain impact (unsafe)" : "Landing prediction";
                rows[2] = coordinates; rows[3] = "ASL: " + altitude;
                rows[4] = biome;
                rows[5] = "Target difference: " + error;
                rows[6] = impact ? "Surface speed: " + DeltaV(surfaceSpeed) : "Max drag: " + maxDrag.ToString("F1") + "g";
                rows[7] = (impact ? "Delta-v to impact: " : "Landing delta-v: ") + DeltaV(deltaV);
                rows[8] = "Forecast age: " + age.ToString("F1") + "s";
                rows[9] = "Contact in: " + time;
            }
            for (int i = 0; i < rows.Length; i++) rows[i] = SingleLine(rows[i] ?? "—");
            return rows;
        }

        internal static double ImpulseAt(IReadOnlyList<AirlessTargetAwareState> path, double ut)
        {
            if (path == null || path.Count == 0 || double.IsNaN(ut)) return double.NaN;
            if (ut < path[0].UT) return 0;
            for (int i = 0; i < path.Count; i++)
            {
                var b = path[i];
                if (ut > b.UT) continue;
                if (ut == b.UT) return b.PropulsiveDeltaV;
                if (i == 0) return 0;
                var a = path[i - 1];
                return a.PropulsiveDeltaV + (b.PropulsiveDeltaV - a.PropulsiveDeltaV) *
                    ((ut - a.UT) / (b.UT - a.UT));
            }
            return double.NaN; // A tail beyond the resolved path is not a total.
        }
    }
}
