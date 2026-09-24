using System;

namespace MuMech
{
    /// <summary>
    /// V2-owned safety gate for staging during an already-authorized powered
    /// descent.  It deliberately cannot stage while parachutes are deployed,
    /// while the command is a fine-throttle command, or before a sustained
    /// loss of measured thrust has been observed.  The flight module supplies
    /// the KSP-specific next-stage capability check and performs the stage.
    /// </summary>
    public sealed class V2PoweredStagingGate
    {
        public const double MinimumCommandedThrottle = 0.90;
        public const double MaximumHealthyThrustFraction = 0.15;
        public const double LossConfirmationSeconds = 0.75;
        private double _lossSinceUT = double.NaN;

        public V2PoweredStageDecision Decide(double ut, bool poweredDescentPhase, bool parachutesDeployed,
            double commandedThrottle, double measuredAcceleration, double maximumAcceleration,
            bool nextStageHasUsablePropulsion)
        {
            if (!poweredDescentPhase || parachutesDeployed || !Finite(ut) ||
                !Finite(commandedThrottle) || commandedThrottle < MinimumCommandedThrottle ||
                !Finite(measuredAcceleration) || !Finite(maximumAcceleration) || maximumAcceleration <= 0)
                return ResetAndHold("V2 staging gate is not in a sustained full-throttle powered descent.");

            // A normally producing engine can read below its nominal maximum
            // during spool-up or as a consequence of the main throttle cap.
            // Only a near-total, sustained loss is allowed to advance staging.
            if (measuredAcceleration > maximumAcceleration * MaximumHealthyThrustFraction)
                return ResetAndHold("V2 staging gate sees measured propulsion.");
            if (!nextStageHasUsablePropulsion)
                return ResetAndHold("V2 staging gate found no usable next powered stage.");

            if (double.IsNaN(_lossSinceUT))
            {
                _lossSinceUT = ut;
                return new V2PoweredStageDecision(V2PoweredStageDirective.Hold,
                    "V2 staging gate is confirming loss of commanded thrust.");
            }
            if (ut - _lossSinceUT < LossConfirmationSeconds)
                return new V2PoweredStageDecision(V2PoweredStageDirective.Hold,
                    "V2 staging gate is confirming loss of commanded thrust.");

            _lossSinceUT = double.NaN;
            return new V2PoweredStageDecision(V2PoweredStageDirective.CommandStage,
                "V2 confirmed loss of full-throttle propulsion and a usable next stage.");
        }

        public void Reset() => _lossSinceUT = double.NaN;

        private V2PoweredStageDecision ResetAndHold(string reason)
        {
            _lossSinceUT = double.NaN;
            return new V2PoweredStageDecision(V2PoweredStageDirective.Hold, reason);
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    public enum V2PoweredStageDirective { Hold, CommandStage }

    public sealed class V2PoweredStageDecision
    {
        public readonly V2PoweredStageDirective Directive;
        public readonly string Reason;

        public V2PoweredStageDecision(V2PoweredStageDirective directive, string reason)
        {
            Directive = directive;
            Reason = reason;
        }
    }
}
