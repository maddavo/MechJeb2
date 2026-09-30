using System;
using System.Collections.Generic;

namespace MuMech.Landing
{
    // Flight-thread-only, demand-loaded PQS samples. Stable body-fixed query
    // locations let advancing snapshots share terrain without preloading a map.
    internal sealed class TargetAwareTerrainCache
    {
        private readonly int _capacity;
        private readonly Dictionary<(double, double), double> _samples =
            new Dictionary<(double, double), double>();
        private readonly Queue<(double, double)> _order = new Queue<(double, double)>();
        private object _body, _terrain;
        private double _minimum, _maximum;
        internal bool SpatialReuseEnabled { get; }

        internal TargetAwareTerrainCache(int capacity = 16384, bool spatialReuseEnabled = true)
        {
            if (capacity <= 0) throw new ArgumentOutOfRangeException(nameof(capacity));
            _capacity = capacity;
            SpatialReuseEnabled = spatialReuseEnabled;
        }

        internal int Count => _samples.Count;

        internal static void QueryLocation(double latitude, double longitude,
            double bodyRadius, double resolutionMetres, out double queryLatitude,
            out double queryLongitude)
        {
            if (!Finite(latitude) || !Finite(longitude) || !Finite(bodyRadius) ||
                !Finite(resolutionMetres) || bodyRadius <= 0 || resolutionMetres <= 0 ||
                latitude < -90 || latitude > 90)
                throw new ArgumentException("Invalid terrain query location");
            // Equal angular cells are no wider than resolutionMetres on the
            // surface; longitude cells become narrower toward the poles.
            double step = resolutionMetres / bodyRadius * 180 / Math.PI;
            longitude = ((longitude + 180) % 360 + 360) % 360 - 180;
            queryLatitude = Math.Max(-90, Math.Min(90, Math.Round(latitude / step) * step));
            queryLongitude = Math.Round(longitude / step) * step;
            queryLongitude = ((queryLongitude + 180) % 360 + 360) % 360 - 180;
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        internal void Configure(object body, object terrain, double minimum,
            double maximum, double targetHeight)
        {
            if (!ReferenceEquals(_body, body) || !ReferenceEquals(_terrain, terrain) ||
                _minimum != minimum || _maximum != maximum)
                Clear();
            _body = body;
            _terrain = terrain;
            _minimum = minimum;
            _maximum = maximum;
            // A changed target does not change any body-fixed PQS height.
        }

        internal bool TryGet(double latitude, double longitude, out double height) =>
            _samples.TryGetValue((latitude, longitude), out height);

        internal void Store(double latitude, double longitude, double height)
        {
            if (double.IsNaN(latitude) || double.IsInfinity(latitude) ||
                double.IsNaN(longitude) || double.IsInfinity(longitude) ||
                double.IsNaN(height) || double.IsInfinity(height))
                throw new ArgumentException("Non-finite terrain sample");
            var key = (latitude, longitude);
            if (_samples.ContainsKey(key)) return;
            if (_samples.Count == _capacity) _samples.Remove(_order.Dequeue());
            _samples.Add(key, height);
            _order.Enqueue(key);
        }

        internal void Clear()
        {
            _samples.Clear();
            _order.Clear();
            _body = _terrain = null;
        }
    }
}
