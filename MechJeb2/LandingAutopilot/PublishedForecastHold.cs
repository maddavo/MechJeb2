using System;

namespace MuMech.Landing
{
    // Sole ownership of an invalidated published payload for observation.
    // This class has no control-readiness or prediction-selection API.
    internal sealed class PublishedForecastHold : IDisposable
    {
        private ReentrySimulation.Result value;
        private object body;
        private double latitude, longitude;

        internal void Retain(ReentrySimulation.Result result, object contextBody, double lat, double lon)
        {
            if (ReferenceEquals(value, result)) return;
            Dispose(); value = result; body = contextBody; latitude = lat; longitude = lon;
        }

        internal ReentrySimulation.Result Observe(object contextBody, double lat, double lon)
        {
            if (!ReferenceEquals(body, contextBody) || latitude != lat || longitude != lon) Dispose();
            return value;
        }

        public void Dispose()
        {
            var old = value; value = null; body = null;
            old?.Release();
        }
    }
}
