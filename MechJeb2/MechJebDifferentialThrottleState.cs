using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace MuMech
{
    // No GUI/control law. ModuleManager adds this to engine and MechJeb parts so
    // baseline data is serialized on the same part and survives vessel splits.
    public sealed class MechJebDifferentialThrottleState : PartModule
    {
        private static readonly HashSet<Guid> DirtyPreferences = new HashSet<Guid>();
        private readonly DifferentialThrottleLimiterLedger _ledger = new DifferentialThrottleLimiterLedger();
        private ConfigNode _unparsedRecovery;
        private MechJebCore _owner;
        private string _ownerToken;
        private Guid _ownerVessel;
        private Guid _observedVessel;
        private int _observedPartCount;
        private bool _preferencePending = true;
        private bool _subscribed;
        private string _preferenceGroup = "";
        private bool _preference;

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            _unparsedRecovery = null;
            _owner = null;
            _ownerToken = null;
            _ownerVessel = Guid.Empty;
            _preferenceGroup = node.GetValue("preferenceGroup") ?? "";
            if (!bool.TryParse(node.GetValue("preferenceEnabled"), out _preference)) _preferenceGroup = "";
            ConfigNode recovery = node.GetNode("LIMITER_RECOVERY");
            if (!_ledger.Load(recovery))
            {
                _unparsedRecovery = recovery;
                Debug.LogWarning("[MechJeb Differential Throttle] Invalid/unknown recovery data retained; limiter writes refused on this part.");
            }
            _preferencePending = true;
        }

        public override void OnSave(ConfigNode node)
        {
            base.OnSave(node);
            node.SetValue("preferenceGroup", _preferenceGroup, true);
            node.SetValue("preferenceEnabled", _preference.ToString(), true);
            node.RemoveNodes("LIMITER_RECOVERY");
            node.AddNode(_unparsedRecovery ?? _ledger.Save());
        }

        public override void OnStart(StartState state)
        {
            base.OnStart(state);
            if (!HighLogic.LoadedSceneIsFlight) return;
            // Saved allocations are never adopted as new user limits. This
            // recovery does not depend on an enabled/current-master controller.
            Restore(null);
            if (vessel != null)
            {
                _observedVessel = vessel.id;
                _observedPartCount = vessel.parts.Count;
                DirtyPreferences.Add(vessel.id);
            }
            GameEvents.onVesselWasModified.Add(OnTopologyChanged);
            _subscribed = true;
        }

        public void FixedUpdate()
        {
            if (!HighLogic.LoadedSceneIsFlight || vessel == null || !FlightGlobals.ready) return;
            if (_observedVessel != vessel.id || _observedPartCount != vessel.parts.Count)
            {
                Restore(null);
                _observedVessel = vessel.id;
                _observedPartCount = vessel.parts.Count;
                _preferencePending = true;
                DirtyPreferences.Add(vessel.id);
            }
            if (_owner == null && _ownerToken != null) Restore(_ownerToken);
            if (_owner != null && (_owner.vessel != vessel || _ownerVessel != vessel.id ||
                vessel.GetMasterMechJeb() != _owner || !_owner.Thrust.Enabled || !_owner.Thrust.DifferentialThrottle))
                Restore(_ownerToken);
            if (_preferencePending)
            {
                MechJebCore master = vessel.GetMasterMechJeb();
                if (master != null) SyncPreference(master);
            }
        }

        private void OnTopologyChanged(Vessel modified)
        {
            if (modified == null || (modified != vessel && modified.id != _ownerVessel)) return;
            Restore(null);
            _preferencePending = true;
            DirtyPreferences.Add(modified.id);
            if (vessel != null) DirtyPreferences.Add(vessel.id);
        }

        public void OnDestroy()
        {
            if (_subscribed) GameEvents.onVesselWasModified.Remove(OnTopologyChanged);
            Restore(_ownerToken);
        }

        internal static MechJebDifferentialThrottleState Find(Part enginePart) =>
            enginePart.FindModuleImplementing<MechJebDifferentialThrottleState>();

        private string Key(ModuleEngines engine)
        {
            int index = 0;
            foreach (ModuleEngines candidate in part.Modules.OfType<ModuleEngines>())
            {
                if (ReferenceEquals(candidate, engine)) return index + ":" + engine.moduleName + ":" + engine.engineID;
                index++;
            }
            return null;
        }

        internal float BaselineLimiter(ModuleEngines engine) =>
            _ledger.Baseline(Key(engine), engine.thrustPercentage, owned: _ownerToken != null);

        internal bool CanApply(ModuleEngines engine, MechJebCore core, float requested) =>
            _unparsedRecovery == null && engine != null && engine.part == part && core != null &&
            core.vessel == vessel && vessel.GetMasterMechJeb() == core && Key(engine) != null &&
            DifferentialThrottleLimiterLedger.Valid(engine.thrustPercentage) && DifferentialThrottleLimiterLedger.Valid(requested);

        internal void Apply(ModuleEngines engine, MechJebCore core, string token, float requested)
        {
            if (!CanApply(engine, core, requested)) throw new InvalidOperationException("Differential-throttle part/owner changed before publication");
            if (_ownerToken != null && (_ownerToken != token || _ownerVessel != vessel.id)) Restore(null);
            _owner = core;
            _ownerToken = token;
            _ownerVessel = vessel.id;
            engine.thrustPercentage = _ledger.Apply(Key(engine), engine.thrustPercentage, requested, token, retainBaseline: true);
        }

        internal void Restore(string owner)
        {
            if (part == null || _unparsedRecovery != null || (owner != null && _ownerToken != owner)) return;
            int count = _ledger.Count;
            foreach (ModuleEngines engine in part.Modules.OfType<ModuleEngines>())
                engine.thrustPercentage = _ledger.Release(Key(engine), engine.thrustPercentage, owner);
            if (_ledger.Count != count)
                Debug.Log("[MechJeb Differential Throttle] Restored/released engine limiter overrides on part " + part.flightID);
            if (owner == null || _ownerToken == owner)
            {
                _owner = null;
                _ownerToken = null;
                _ownerVessel = Guid.Empty;
            }
        }

        // Called on topology/load or a mode change, not on every solver update.
        internal static bool PreferenceDirty(Vessel v) => v != null && DirtyPreferences.Contains(v.id);

        internal static void SyncPreference(MechJebCore core, bool restoreSaved = true, bool forceRestore = false)
        {
            if (core == null || core.vessel == null || core.vessel.GetMasterMechJeb() != core) return;
            var states = core.vessel.parts.SelectMany(p => p.Modules.OfType<MechJebDifferentialThrottleState>()).ToArray();
            if (states.Length == 0) return;
            bool pending = states.Any(s => s._preferencePending);
            var known = states.Where(s => s._preferenceGroup.Length != 0).ToArray();
            if (restoreSaved && (pending || forceRestore) && known.Length == states.Length && DifferentialThrottleLimiterLedger.TryPreference(
                known.Select(s => new KeyValuePair<string, bool>(s._preferenceGroup, s._preference)), out bool recovered))
                core.Thrust.DifferentialThrottle = recovered;

            string group = known.Length != 0 && known.All(s => s._preferenceGroup == known[0]._preferenceGroup)
                ? known[0]._preferenceGroup : core.vessel.id.ToString("N");
            foreach (var state in states)
            {
                if (state._preferenceGroup.Length == 0)
                {
                    state._preferenceGroup = group;
                    state._preference = core.Thrust.DifferentialThrottle;
                }
                state._preferencePending = false;
            }
            if (states.All(s => s._preferenceGroup == states[0]._preferenceGroup))
                foreach (var state in states) state._preference = core.Thrust.DifferentialThrottle;
            DirtyPreferences.Remove(core.vessel.id);
            // Mixed docking constituents deliberately retain their own mode
            // preferences; the current master governs only the merged vessel.
        }
    }
}
