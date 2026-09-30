using System;

namespace MuMech.Landing
{
    // Pure phase/readiness boundary shared by the live V1 consumers and replay.
    internal static class LandingForecastContract
    {
        internal static ReentrySimulation.LandingForecastKind FreshState(
            ReentrySimulation.LandingForecastKind committed, double inputUT,
            double currentUT, double maximumAge, bool settled)
        {
            if (!settled || !Finite(inputUT) || !Finite(currentUT) ||
                !Finite(maximumAge) || maximumAge < 0 || inputUT > currentUT ||
                currentUT - inputUT > maximumAge)
                return ReentrySimulation.LandingForecastKind.NoForecast;
            return committed;
        }

        internal static bool CorrectionReady(ReentrySimulation.LandingForecastKind kind) =>
            kind == ReentrySimulation.LandingForecastKind.ImpactForecast ||
            kind == ReentrySimulation.LandingForecastKind.LandableForecast;

        internal static bool LandingReady(ReentrySimulation.LandingForecastKind kind) =>
            kind == ReentrySimulation.LandingForecastKind.LandableForecast;

        // This is an impossibility deadline, not a retry counter. The attitude
        // and pulse terms are physical lower bounds; the remaining terms are
        // V1's required settle and independent-result cadence.
        internal static bool CorrectionCycleImpossible(double timeToImpact,
            double minimumAttitudeTime, double minimumPulseTime,
            double settlingTime, double refreshInterval, double measuredLatency)
        {
            if (!Finite(timeToImpact) || !Finite(minimumAttitudeTime) ||
                !Finite(minimumPulseTime) || !Finite(settlingTime) ||
                !Finite(refreshInterval) || !Finite(measuredLatency) ||
                minimumAttitudeTime < 0 || minimumPulseTime < 0 ||
                settlingTime < 0 || refreshInterval < 0 || measuredLatency < 0)
                return true;
            return timeToImpact <= minimumAttitudeTime + minimumPulseTime +
                settlingTime + 2 * (refreshInterval + measuredLatency);
        }

        private static bool Finite(double value) =>
            !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
