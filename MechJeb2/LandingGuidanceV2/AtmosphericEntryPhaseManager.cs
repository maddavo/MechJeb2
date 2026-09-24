using System;

namespace MuMech
{
    /// <summary>
    /// Owns authority transitions for V2's finite atmospheric-entry burn. It
    /// deliberately contains no vessel or Unity access, so the live controller
    /// and deterministic harness execute the same warp, attitude, validation,
    /// and finite-burn sequence.
    /// </summary>
    public sealed class AtmosphericEntryPhaseManager
    {
        public const double AttitudeReadyDegrees = AirlessLandingPhaseManager.AttitudeReadyDegrees;
        public const double InitialWarpLeadSeconds = AirlessLandingPhaseManager.InitialWarpLeadSeconds;
        public const double WarpSettleMargin = AirlessLandingPhaseManager.WarpSettleMargin;
        public const double BurnCompleteDeltaV = AirlessLandingPhaseManager.BurnCompleteDeltaV;

        private AtmosphericLandingPlan _plan;
        private readonly double _finiteBurnLeadSeconds;
        private bool _burnGateValidated;

        public AtmosphericEntryPhase Phase { get; private set; } = AtmosphericEntryPhase.Idle;
        public int LastWorkUnits { get; private set; }
        public AtmosphericLandingPlan Plan => _plan;
        public double IgnitionUT => _plan == null ? double.NaN : _plan.StrategicEntryBurnUT - _finiteBurnLeadSeconds;

        public AtmosphericEntryPhaseManager(double finiteBurnLeadSeconds)
        {
            _finiteBurnLeadSeconds = Finite(finiteBurnLeadSeconds) && finiteBurnLeadSeconds >= 0
                ? finiteBurnLeadSeconds : 0;
        }

        public AtmosphericEntryPhaseDecision Start(AtmosphericLandingPlan plan)
        {
            _plan = plan;
            _burnGateValidated = false;
            if (!Executable(plan, out string reason)) return Reject(reason);
            if (plan.StrategicEntryDeltaV.magnitude <= BurnCompleteDeltaV)
            {
                Phase = AtmosphericEntryPhase.Entry;
                return Decision(AtmosphericEntryDirective.EnterAtmosphericEntry);
            }
            Phase = AtmosphericEntryPhase.InitialWarpToBurn;
            return Decision(AtmosphericEntryDirective.None);
        }

        public AtmosphericEntryPhaseDecision Tick(double ut, bool autoWarp, double attitudeErrorDegrees,
            double remainingBurnDeltaV)
        {
            if (_plan == null) return Reject("V2 atmospheric phase manager has no active entry plan.");
            switch (Phase)
            {
                case AtmosphericEntryPhase.InitialWarpToBurn:
                    if (autoWarp && ut < IgnitionUT - InitialWarpLeadSeconds)
                        return Decision(AtmosphericEntryDirective.RequestInitialWarp, IgnitionUT - InitialWarpLeadSeconds);
                    Phase = AtmosphericEntryPhase.PrepareWarp;
                    return Decision(AtmosphericEntryDirective.RequestAttitude);
                case AtmosphericEntryPhase.PrepareWarp:
                    if (attitudeErrorDegrees > AttitudeReadyDegrees)
                        return Decision(AtmosphericEntryDirective.RequestAttitude);
                    Phase = AtmosphericEntryPhase.WarpToBurn;
                    return Decision(AtmosphericEntryDirective.WarpAuthorized);
                case AtmosphericEntryPhase.WarpToBurn:
                    if (autoWarp && ut < IgnitionUT - WarpSettleMargin)
                        return Decision(AtmosphericEntryDirective.RequestWarp, IgnitionUT - WarpSettleMargin);
                    Phase = AtmosphericEntryPhase.AlignBurn;
                    _burnGateValidated = false;
                    return Decision(AtmosphericEntryDirective.ExitWarpAndRequestAttitude);
                case AtmosphericEntryPhase.AlignBurn:
                    if (ut < IgnitionUT) return Decision(AtmosphericEntryDirective.RequestAttitude);
                    if (!_burnGateValidated) return Decision(AtmosphericEntryDirective.RequireFreshBurnValidation);
                    if (attitudeErrorDegrees > AttitudeReadyDegrees)
                        return Decision(AtmosphericEntryDirective.RequestAttitude);
                    Phase = AtmosphericEntryPhase.Burn;
                    return Decision(AtmosphericEntryDirective.BeginFiniteBurn);
                case AtmosphericEntryPhase.Burn:
                    if (remainingBurnDeltaV > BurnCompleteDeltaV)
                    {
                        if (attitudeErrorDegrees > AttitudeReadyDegrees)
                            return Decision(AtmosphericEntryDirective.RequestAttitude);
                        return Decision(AtmosphericEntryDirective.RequestFiniteBurnThrottle);
                    }
                    Phase = AtmosphericEntryPhase.AwaitPostBurnValidation;
                    return Decision(AtmosphericEntryDirective.FiniteBurnComplete);
                case AtmosphericEntryPhase.AwaitPostBurnValidation:
                    return Decision(AtmosphericEntryDirective.RequirePostBurnValidation);
                case AtmosphericEntryPhase.Entry:
                    return Decision(AtmosphericEntryDirective.EnterAtmosphericEntry);
                case AtmosphericEntryPhase.Rejected:
                    return Decision(AtmosphericEntryDirective.None);
                default:
                    return Reject("V2 atmospheric phase manager entered an unknown phase.");
            }
        }

        public AtmosphericEntryPhaseDecision AcceptFreshBurnValidation(long snapshotVersion, double snapshotUT,
            bool valid, string rejectionReason = null)
        {
            if (Phase != AtmosphericEntryPhase.AlignBurn)
                return Reject("A V2 atmospheric ignition validation was supplied outside the entry-burn alignment phase.");
            if (snapshotUT < IgnitionUT)
                return Reject("V2 atmospheric ignition validation arrived before the finite-burn ignition gate.");
            if (snapshotUT > _plan.StrategicEntryBurnUT + 0.25)
                return Reject("V2 atmospheric ignition validation missed the planned finite-burn midpoint.");
            if (snapshotVersion <= _plan.SnapshotVersion)
                return Reject("V2 atmospheric ignition validation did not use a fresh post-warp snapshot.");
            if (!valid)
                return Reject(string.IsNullOrEmpty(rejectionReason)
                    ? "V2 atmospheric ignition validation rejected the committed entry burn."
                    : rejectionReason);
            _burnGateValidated = true;
            return Decision(AtmosphericEntryDirective.RequestAttitude);
        }

        public AtmosphericEntryPhaseDecision AcceptFreshBurnValidation(AtmosphericLandingPlan plan, double snapshotUT)
        {
            if (plan == null) return Reject("V2 atmospheric ignition validation did not provide a fresh entry plan.");
            AtmosphericEntryPhaseDecision decision = AcceptFreshBurnValidation(plan.SnapshotVersion, snapshotUT,
                plan.State == AtmosphericLandingPlanState.Candidate, plan.Reason);
            if (decision.Directive == AtmosphericEntryDirective.Reject) return decision;
            _plan = plan;
            // A fresh simulation may show that the actual current orbit has
            // already crossed the atmospheric interface. In that case a
            // second strategic burn is neither required nor safe to force;
            // enter the separately validated atmospheric profile directly.
            if (plan.StrategicEntryDeltaV.magnitude <= BurnCompleteDeltaV)
            {
                Phase = AtmosphericEntryPhase.Entry;
                return Decision(AtmosphericEntryDirective.EnterAtmosphericEntry);
            }
            return decision;
        }

        public AtmosphericEntryPhaseDecision AcceptPostBurnValidation(AtmosphericLandingPlan plan)
        {
            if (Phase != AtmosphericEntryPhase.AwaitPostBurnValidation)
                return Reject("A V2 atmospheric post-burn plan was supplied outside its validation gate.");
            if (!Executable(plan, out string reason)) return Reject(reason);
            if (plan.SnapshotVersion <= _plan.SnapshotVersion)
                return Reject("V2 atmospheric post-burn validation did not use a fresh snapshot.");
            _plan = plan;
            _burnGateValidated = false;
            if (plan.StrategicEntryDeltaV.magnitude <= BurnCompleteDeltaV)
            {
                Phase = AtmosphericEntryPhase.Entry;
                return Decision(AtmosphericEntryDirective.EnterAtmosphericEntry);
            }
            Phase = AtmosphericEntryPhase.InitialWarpToBurn;
            return Decision(AtmosphericEntryDirective.None);
        }

        private static bool Executable(AtmosphericLandingPlan plan, out string reason)
        {
            if (plan == null || plan.State != AtmosphericLandingPlanState.Candidate)
            {
                reason = "V2 requires a candidate atmospheric entry plan.";
                return false;
            }
            if (!Finite(plan.LandingMargin) || plan.LandingMargin < 0)
            {
                reason = "V2 atmospheric entry plan does not preserve its powered terminal reserve.";
                return false;
            }
            if (plan.StrategicEntryDeltaV.magnitude > BurnCompleteDeltaV &&
                (!Finite(plan.StrategicEntryBurnUT) || plan.StrategicEntryBurnUT <= 0))
            {
                reason = "V2 atmospheric entry plan has no finite strategic-burn epoch.";
                return false;
            }
            reason = null;
            return true;
        }

        private AtmosphericEntryPhaseDecision Reject(string reason)
        {
            Phase = AtmosphericEntryPhase.Rejected;
            LastWorkUnits = 1;
            return new AtmosphericEntryPhaseDecision(Phase, AtmosphericEntryDirective.Reject, double.NaN, reason, LastWorkUnits);
        }

        private AtmosphericEntryPhaseDecision Decision(AtmosphericEntryDirective directive, double warpUT = double.NaN)
        {
            LastWorkUnits = 1;
            return new AtmosphericEntryPhaseDecision(Phase, directive, warpUT, null, LastWorkUnits);
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    public enum AtmosphericEntryPhase
    {
        Idle, InitialWarpToBurn, PrepareWarp, WarpToBurn, AlignBurn, Burn, AwaitPostBurnValidation, Entry, Rejected
    }

    public enum AtmosphericEntryDirective
    {
        None, RequestInitialWarp, RequestAttitude, WarpAuthorized, RequestWarp, ExitWarpAndRequestAttitude,
        RequireFreshBurnValidation, BeginFiniteBurn, RequestFiniteBurnThrottle, FiniteBurnComplete,
        RequirePostBurnValidation, EnterAtmosphericEntry, Reject
    }

    public sealed class AtmosphericEntryPhaseDecision
    {
        public readonly AtmosphericEntryPhase Phase;
        public readonly AtmosphericEntryDirective Directive;
        public readonly double WarpUT;
        public readonly string Reason;
        public readonly int WorkUnits;

        public AtmosphericEntryPhaseDecision(AtmosphericEntryPhase phase, AtmosphericEntryDirective directive,
            double warpUT, string reason, int workUnits)
        {
            Phase = phase;
            Directive = directive;
            WarpUT = warpUT;
            Reason = reason;
            WorkUnits = workUnits;
        }
    }
}
