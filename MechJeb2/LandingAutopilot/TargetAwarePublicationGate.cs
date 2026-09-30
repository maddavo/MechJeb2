using System;

namespace MuMech.Landing
{
    // Immutable lineage accompanies, but never changes, the V1-compatible
    // ReentrySimulation.Result committed to the predictor's public slot.
    internal readonly struct TargetAwareResultLineage
    {
        internal readonly long Generation;
        internal readonly long Sequence;
        internal readonly object Body;
        internal readonly double TargetLatitude;
        internal readonly double TargetLongitude;
        internal readonly double TargetTerrainASL;
        internal readonly double CaptureUT;
        internal readonly double InputUT;
        internal readonly bool TargetAwareModel;
        internal readonly bool Complete;
        internal readonly bool TerrainResolved;
        internal readonly bool ClearPath;
        internal readonly bool TerminalNecessaryBoundPasses;
        internal readonly bool TerminalTouchdownValidated;
        internal readonly ReentrySimulation.LandingForecastKind ForecastKind;
        internal readonly bool HasFirstContact;

        internal TargetAwareResultLineage(long generation, long sequence, object body,
            double targetLatitude, double targetLongitude, double targetTerrainASL,
            double captureUT, double inputUT,
            bool targetAwareModel, bool complete, bool terrainResolved, bool clearPath,
            bool terminalNecessaryBoundPasses,
            ReentrySimulation.LandingForecastKind forecastKind,
            bool hasFirstContact,
            bool terminalTouchdownValidated)
        {
            Generation = generation;
            Sequence = sequence;
            Body = body;
            TargetLatitude = targetLatitude;
            TargetLongitude = targetLongitude;
            TargetTerrainASL = targetTerrainASL;
            CaptureUT = captureUT;
            InputUT = inputUT;
            TargetAwareModel = targetAwareModel;
            Complete = complete;
            TerrainResolved = terrainResolved;
            ClearPath = clearPath;
            TerminalNecessaryBoundPasses = terminalNecessaryBoundPasses;
            TerminalTouchdownValidated = terminalTouchdownValidated;
            ForecastKind = forecastKind;
            HasFirstContact = hasFirstContact;
        }
    }

    internal enum TargetAwarePublicationDecision
    {
        Accept,
        WrongModel,
        Incomplete,
        TerrainUnresolved,
        TerrainIntersection,
        TerminalNecessaryBoundFailed,
        ImpactOnly,
        StaleGeneration,
        ChangedBodyOrTarget,
        ExpiredSnapshot,
        OutOfOrder
    }

    internal static class TargetAwarePublicationGate
    {
        internal static bool CanSeedSearch(TargetAwareResultLineage? committed,
            bool directForecast, long currentGeneration, object currentBody,
            double targetLatitude, double targetLongitude, double targetTerrainASL,
            double currentUT, double maximumSnapshotAge) =>
            !directForecast && committed.HasValue &&
            Check(committed.Value, null, currentGeneration, currentBody,
                targetLatitude, targetLongitude, targetTerrainASL,
                currentUT, maximumSnapshotAge, 0) == TargetAwarePublicationDecision.Accept;

        internal static TargetAwarePublicationDecision Check(TargetAwareResultLineage candidate,
            TargetAwareResultLineage? committed, long currentGeneration, object currentBody,
            double currentTargetLatitude, double currentTargetLongitude, double currentTargetTerrainASL,
            double currentUT, double maximumSnapshotAge, double maximumTerrainChange)
        {
            if (!candidate.TargetAwareModel) return TargetAwarePublicationDecision.WrongModel;
            // V1 Beta's sole readiness gate accepts Outcome.LANDED. An impact
            // may be captured for diagnosis but cannot own its active slot.
            if (candidate.ForecastKind == ReentrySimulation.LandingForecastKind.ImpactForecast)
                return TargetAwarePublicationDecision.ImpactOnly;
            if ((candidate.ForecastKind != ReentrySimulation.LandingForecastKind.ImpactForecast &&
                 candidate.ForecastKind != ReentrySimulation.LandingForecastKind.LandableForecast) ||
                (candidate.ForecastKind == ReentrySimulation.LandingForecastKind.ImpactForecast &&
                 (!candidate.HasFirstContact || candidate.TerminalTouchdownValidated)) ||
                (candidate.ForecastKind == ReentrySimulation.LandingForecastKind.LandableForecast &&
                 (!candidate.TerminalTouchdownValidated || candidate.HasFirstContact)))
                return TargetAwarePublicationDecision.Incomplete;
            if (!candidate.Complete) return TargetAwarePublicationDecision.Incomplete;
            if (!candidate.TerrainResolved) return TargetAwarePublicationDecision.TerrainUnresolved;
            if (!candidate.ClearPath) return TargetAwarePublicationDecision.TerrainIntersection;
            if (candidate.ForecastKind ==
                ReentrySimulation.LandingForecastKind.LandableForecast &&
                !candidate.TerminalNecessaryBoundPasses)
                return TargetAwarePublicationDecision.TerminalNecessaryBoundFailed;
            if (candidate.Generation != currentGeneration)
                return TargetAwarePublicationDecision.StaleGeneration;
            if (candidate.Body == null || !ReferenceEquals(candidate.Body, currentBody) ||
                !Finite(candidate.TargetLatitude) || !Finite(candidate.TargetLongitude) ||
                !Finite(candidate.TargetTerrainASL) || !Finite(currentTargetTerrainASL) ||
                candidate.TargetLatitude != currentTargetLatitude ||
                candidate.TargetLongitude != currentTargetLongitude ||
                !Finite(maximumTerrainChange) || maximumTerrainChange < 0 ||
                Math.Abs(candidate.TargetTerrainASL - currentTargetTerrainASL) > maximumTerrainChange)
                return TargetAwarePublicationDecision.ChangedBodyOrTarget;
            if (!Finite(currentUT) || !Finite(candidate.CaptureUT) || !Finite(candidate.InputUT) ||
                !Finite(maximumSnapshotAge) || maximumSnapshotAge < 0 ||
                candidate.CaptureUT > currentUT || currentUT - candidate.CaptureUT > maximumSnapshotAge)
                return TargetAwarePublicationDecision.ExpiredSnapshot;
            if (candidate.Sequence <= 0 || (committed.HasValue &&
                (candidate.Sequence <= committed.Value.Sequence ||
                 candidate.CaptureUT <= committed.Value.CaptureUT)))
                return TargetAwarePublicationDecision.OutOfOrder;
            return TargetAwarePublicationDecision.Accept;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
