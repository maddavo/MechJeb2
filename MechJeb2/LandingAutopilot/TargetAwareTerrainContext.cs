using System;

namespace MuMech.Landing
{
    // Controller configuration, not a fresh endpoint or a terrain cache entry.
    // Direct forecasts retain this policy even when contact occurs elsewhere.
    internal sealed class TargetAwareBrakingPolicyContext
    {
        private object _body;
        private long _generation, _sequence;
        private double _latitude, _longitude, _sourceUT;
        private double _targetTerrainASL = double.NaN;
        private bool _postPulseRevalidation;
        internal long PlanId { get; private set; }
        internal double TerrainASL { get; private set; }
        internal double ReferenceUT { get; private set; }

        internal bool Commit(object body, long generation, double latitude, double longitude,
            long sequence, double inputUT, double terrainASL, double referenceUT, bool retainExecutedPlan,
            double targetTerrainASL = double.NaN)
        {
            if (body == null || sequence <= 0 || !Finite(inputUT) || !Finite(terrainASL) ||
                !Finite(referenceUT) || !Finite(latitude) || !Finite(longitude)) return false;
            bool same = Matches(body, generation, latitude, longitude);
            if (same && (sequence <= _sequence || inputUT < _sourceUT)) return false;
            if (same && retainExecutedPlan)
            {
                if (terrainASL != TerrainASL) return false;
                // The refreshed first-thrust prediction can move; the executed
                // reference and plan identity cannot be rewritten retrospectively.
                _sequence = sequence;
                _sourceUT = inputUT;
                _postPulseRevalidation = false;
                return true;
            }
            _body = body; _generation = generation; _latitude = latitude; _longitude = longitude;
            _sequence = PlanId = sequence; _sourceUT = inputUT;
            TerrainASL = terrainASL; ReferenceUT = referenceUT;
            _targetTerrainASL = targetTerrainASL;
            _postPulseRevalidation = false;
            return true;
        }

        // A real correction destroys trajectory authority, not the nominal
        // control configuration the correction was calculated against. This
        // retains a search hint only; a changed-state worker must validate it.
        internal void InvalidateForecastAfterPulse() => _postPulseRevalidation = _body != null;

        internal bool TryGetPostPulseSeed(object body, long generation, double latitude,
            double longitude, double targetTerrainASL, double now,
            out double referenceUT, out double terrainASL)
        {
            referenceUT = double.NaN; terrainASL = double.NaN;
            if (!_postPulseRevalidation) return false;
            if (!Matches(body,generation,latitude,longitude) || !Finite(now) || now < _sourceUT ||
                !Finite(_targetTerrainASL) || targetTerrainASL != _targetTerrainASL)
            {
                Clear();
                return false;
            }
            referenceUT = ReferenceUT; terrainASL = TerrainASL;
            return true;
        }

        internal bool TryGet(object body, long generation, double latitude, double longitude,
            double now, out double terrainASL)
        {
            terrainASL = TerrainASL;
            if (Finite(now) && now < _sourceUT) Clear();
            return Matches(body, generation, latitude, longitude);
        }

        private bool Matches(object body, long generation, double latitude, double longitude) =>
            _body != null && ReferenceEquals(body, _body) && generation == _generation &&
            latitude == _latitude && longitude == _longitude;
        internal void Clear() { _body = null; _sequence = PlanId = 0; _postPulseRevalidation = false; }
        private static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
    }

    // A terrain observation is not trajectory readiness. Keep its body/target
    // ownership while an asynchronous refresh or attitude observation is pending.
    internal sealed class TargetAwareTerrainContext
    {
        private object _body;
        private double _latitude, _longitude, _height;
        private long _sequence;

        internal bool Observe(object body, double latitude, double longitude,
            double height, long sequence)
        {
            if (body == null || !Finite(latitude) || !Finite(longitude) ||
                !Finite(height) || sequence <= 0) return false;
            if (ReferenceEquals(body, _body) && latitude == _latitude &&
                longitude == _longitude && sequence < _sequence) return false;
            _body = body; _latitude = latitude; _longitude = longitude;
            _height = height; _sequence = sequence;
            return true;
        }

        internal bool TryGet(object body, double latitude, double longitude, out double height)
        {
            height = _height;
            return _body != null && ReferenceEquals(body, _body) &&
                latitude == _latitude && longitude == _longitude;
        }

        internal void Clear() { _body = null; _sequence = 0; }

        internal static bool ContactValid(AirlessTargetAwareSnapshot snapshot,
            AirlessTargetAwareState contact, double terrainASL) =>
            Finite(contact.UT) && contact.UT >= snapshot.StateUT &&
            Finite(contact.Position.x) && Finite(contact.Position.y) && Finite(contact.Position.z) &&
            Finite(contact.Velocity.x) && Finite(contact.Velocity.y) && Finite(contact.Velocity.z) &&
            Finite(contact.Position.magnitude) &&
            contact.Position.magnitude >= snapshot.BodyRadius + snapshot.MinimumTerrainASL &&
            Finite(terrainASL) && terrainASL >= snapshot.MinimumTerrainASL &&
            terrainASL <= snapshot.MaximumTerrainASL;

        private static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
    }
}
