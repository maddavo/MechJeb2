using System;

namespace MuMech
{
    /// <summary>
    /// Pure V2-only policy for local target changes. The live module supplies
    /// vessel geometry and delta-v; this policy makes the reserve rule
    /// deterministic and independently testable.
    /// </summary>
    public static class V2LocalDivertGate
    {
        public static V2LocalDivertDecision Decide(double remainingDeltaV, double protectedReserve,
            double moveDistance, double timeToGround)
        {
            if (!Finite(remainingDeltaV) || !Finite(protectedReserve) || !Finite(moveDistance) ||
                !Finite(timeToGround) || protectedReserve < 0 || moveDistance < 0 || timeToGround <= 0)
                return new V2LocalDivertDecision(false, double.NaN,
                    "V2 target movement rejected: local divert inputs are not physically valid.");

            double cost = 4.0 * moveDistance / Math.Max(3.0, timeToGround);
            if (remainingDeltaV < protectedReserve + cost)
                return new V2LocalDivertDecision(false, cost,
                    "V2 target movement rejected: it would consume the protected terminal-divert reserve.");
            return new V2LocalDivertDecision(true, cost, null);
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    public sealed class V2LocalDivertDecision
    {
        public readonly bool Accepted;
        public readonly double Cost;
        public readonly string Reason;
        public V2LocalDivertDecision(bool accepted, double cost, string reason)
        { Accepted = accepted; Cost = cost; Reason = reason; }
    }

    /// <summary>
    /// Pure V2 local-site screen. It deliberately rejects uncertainty rather
    /// than changing the player's active target behind their back.
    /// </summary>
    public static class V2LocalSiteGate
    {
        public static V2LocalSiteDecision Assess(bool hasOcean, double terrainASL, double slopeDegrees,
            double roughnessMeters)
        {
            if (!Finite(terrainASL) || !Finite(slopeDegrees) || !Finite(roughnessMeters))
                return new V2LocalSiteDecision(false, "local terrain data is incomplete.");
            if (hasOcean && terrainASL <= 0)
                return new V2LocalSiteDecision(false, "target is below sea level on an ocean body.");
            if (slopeDegrees > 15.0)
                return new V2LocalSiteDecision(false, "local slope exceeds 15 degrees.");
            if (roughnessMeters > 15.0)
                return new V2LocalSiteDecision(false,
                    "local terrain varies by more than 15 m across the footprint sample.");
            return new V2LocalSiteDecision(true, null);
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    public sealed class V2LocalSiteDecision
    {
        public readonly bool Accepted;
        public readonly string Reason;
        public V2LocalSiteDecision(bool accepted, string reason)
        { Accepted = accepted; Reason = reason; }
    }
}
