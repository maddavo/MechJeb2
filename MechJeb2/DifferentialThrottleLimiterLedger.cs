using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace MuMech
{
    // Part-local recovery data, independent of vessel IDs and controller lifetime.
    internal sealed class DifferentialThrottleLimiterLedger
    {
        internal sealed class Entry
        {
            internal string Engine;
            internal float Baseline, Applied;
            internal string Owner;
            internal Entry(string engine, float baseline, float applied, string owner)
            { Engine = engine; Baseline = baseline; Applied = applied; Owner = owner; }
        }

        private readonly Dictionary<string, Entry> _entries = new Dictionary<string, Entry>();
        internal IEnumerable<Entry> Entries => _entries.Values;
        internal int Count => _entries.Count;
        internal static bool Valid(float value) => !float.IsNaN(value) && !float.IsInfinity(value) && value >= 0 && value <= 100;

        internal float Apply(string engine, float current, float requested, string owner)
        {
            if (string.IsNullOrEmpty(engine) || string.IsNullOrEmpty(owner) || !Valid(current) || !Valid(requested))
                throw new ArgumentException("Invalid differential-throttle limiter transaction");
            if (_entries.TryGetValue(engine, out Entry entry))
            {
                // If another writer changed the limiter, that new value is the
                // user's baseline. Otherwise retain the original across refreshes
                // and ownership handoff; never capture a solver allocation.
                if (current != entry.Applied) entry.Baseline = current;
                entry.Applied = requested;
                entry.Owner = owner;
            }
            else _entries.Add(engine, new Entry(engine, current, requested, owner));
            return requested;
        }

        internal float Release(string engine, float current, string owner = null)
        {
            if (!_entries.TryGetValue(engine, out Entry entry) || (owner != null && entry.Owner != owner)) return current;
            _entries.Remove(engine);
            // Do not overwrite a later manual/mod change during cleanup.
            return current == entry.Applied ? entry.Baseline : current;
        }

        internal ConfigNode Save()
        {
            var node = new ConfigNode("LIMITER_RECOVERY");
            node.AddValue("version", "1");
            foreach (Entry entry in _entries.Values)
            {
                ConfigNode child = node.AddNode("ENGINE");
                child.AddValue("key", entry.Engine);
                child.AddValue("baseline", entry.Baseline.ToString("R", CultureInfo.InvariantCulture));
                child.AddValue("applied", entry.Applied.ToString("R", CultureInfo.InvariantCulture));
            }
            return node;
        }

        internal bool Load(ConfigNode node)
        {
            if (node == null) { _entries.Clear(); return true; }
            if (node.GetValue("version") != "1") return false;
            var loaded = new Dictionary<string, Entry>();
            foreach (ConfigNode child in node.GetNodes("ENGINE"))
            {
                string key = child.GetValue("key");
                if (string.IsNullOrEmpty(key) || loaded.ContainsKey(key) ||
                    !float.TryParse(child.GetValue("baseline"), NumberStyles.Float, CultureInfo.InvariantCulture, out float baseline) ||
                    !float.TryParse(child.GetValue("applied"), NumberStyles.Float, CultureInfo.InvariantCulture, out float applied) ||
                    !Valid(baseline) || !Valid(applied)) return false;
                loaded.Add(key, new Entry(key, baseline, applied, "loaded"));
            }
            _entries.Clear();
            foreach (var pair in loaded) _entries.Add(pair.Key, pair.Value);
            return true;
        }

        internal static bool TryPreference(IEnumerable<KeyValuePair<string, bool>> preferences, out bool enabled)
        {
            var values = preferences.ToArray();
            enabled = values.Length != 0 && values[0].Value;
            // A merged vessel retains its current master's mode. Only a
            // homogeneous separated craft can recover a constituent preference.
            bool expected = enabled;
            return values.Length != 0 && values.All(x => x.Key == values[0].Key && x.Value == expected);
        }
    }
}
