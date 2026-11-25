using System;
using System.Collections.Generic;

namespace MuMech.Landing
{
    // Uses completed allocations, sampled on the flight thread. A worker
    // receives only the resulting force/flow values, never mutable engines.
    internal sealed class SustainedBrakingThrust
    {
        internal const double WindowSeconds = 2;
        private readonly Queue<(double UT, double Force, double Flow)> _samples =
            new Queue<(double, double, double)>();
        private double _lastUT = double.NaN;
        private ulong _configuration;
        private double _forceSum, _flowSum;
        internal long Epoch { get; private set; }
        internal int Count => _samples.Count;
        internal double Force => Count > 0 ? _forceSum / Count : double.NaN;
        internal double Flow => Count > 0 ? _flowSum / Count : double.NaN;

        internal void Observe(double ut, ulong configuration, double force, double flow, bool active, bool applied = true)
        {
            if (configuration != _configuration || ut < _lastUT)
            {
                Clear();
                _configuration = configuration;
                Epoch++;
            }
            if (!active || (applied && (!Finite(force) || force <= 0 || !Finite(flow) || flow < 0)))
            {
                Clear();
                return;
            }
            if (!applied)
            {
                // Brief throttle gates provide no new allocation sample.
                if (ut - _lastUT > WindowSeconds) Clear();
                return;
            }
            if (ut == _lastUT) return;
            _lastUT = ut;
            _samples.Enqueue((ut, force, flow));
            _forceSum += force;
            _flowSum += flow;
            while (_samples.Peek().UT < ut - WindowSeconds)
            {
                var removed = _samples.Dequeue();
                _forceSum -= removed.Force;
                _flowSum -= removed.Flow;
            }
        }

        internal bool HasRecentSample(double now) =>
            Count > 0 && now >= _lastUT && now - _lastUT <= WindowSeconds;

        internal double SpeedPolicyAcceleration(double now, double mass, double minimumForce,
            double throttleCap, double liveAcceleration)
        {
            if (!HasRecentSample(now) || !Finite(mass) || mass <= 0 ||
                !Finite(minimumForce) || !Finite(throttleCap) ||
                !Finite(liveAcceleration) || liveAcceleration <= 0)
                return liveAcceleration;
            return (minimumForce + throttleCap * (Force - minimumForce)) / mass;
        }

        private void Clear()
        {
            _samples.Clear();
            _forceSum = _flowSum = 0;
            _lastUT = double.NaN;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
