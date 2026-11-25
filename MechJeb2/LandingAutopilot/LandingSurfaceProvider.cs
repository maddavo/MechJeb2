using System;
using System.Reflection;
using UnityEngine;

namespace MuMech.Landing
{
    // Flight-thread only. Measures loaded KK collision geometry without changing
    // PQS, spawning statics, or retaining world positions across floating origins.
    internal sealed class LandingSurfaceProvider
    {
        private readonly RaycastHit[] _hits = new RaycastHit[64];
        private readonly RaycastHit[] _overflowHits = new RaycastHit[256];
        private Type _kkColliderMarker;
        private bool _lookedForKK;

        internal bool LastQueryUsedDeck { get; private set; }
        internal bool LastQuerySaturated { get; private set; }
        internal double LastDeckASL { get; private set; }

        internal bool Available
        {
            get
            {
                FindKK();
                return _kkColliderMarker != null;
            }
        }

        internal static double Clearance(double naturalClearance, double terrainASL, double surfaceASL)
            => surfaceASL > terrainASL ? Math.Max(0, naturalClearance - (surfaceASL - terrainASL)) : naturalClearance;

        internal double Altitude(CelestialBody body, double latitude, double longitude, double terrainASL)
        {
            LastQueryUsedDeck = LastQuerySaturated = false;
            LastDeckASL = terrainASL;
            FindKK();
            if (_kkColliderMarker == null || body == null || double.IsNaN(terrainASL) || double.IsInfinity(terrainASL))
                return terrainASL;

            Vector3d origin = body.GetWorldSurfacePosition(latitude, longitude, terrainASL + 600);
            Vector3d up = (origin - body.position).normalized;
            var ray = new Ray((Vector3)origin, (Vector3)(-up));
            RaycastHit[] hits = _hits;
            int count = Physics.RaycastNonAlloc(ray, hits, 601, ~0, QueryTriggerInteraction.Ignore);
            if (count == hits.Length)
            {
                Array.Clear(hits, 0, hits.Length);
                hits = _overflowHits;
                count = Physics.RaycastNonAlloc(ray, hits, 601, ~0, QueryTriggerInteraction.Ignore);
            }
            try
            {
                if (count == hits.Length)
                {
                    LastQuerySaturated = true;
                    return terrainASL;
                }
                for (int i = 0; i < count; i++)
                {
                    Collider collider = hits[i].collider;
                    if (collider == null || collider.isTrigger ||
                        collider.GetComponentInParent<Part>() != null ||
                        collider.GetComponentInParent(_kkColliderMarker) == null ||
                        Vector3d.Dot((Vector3d)hits[i].normal, up) < 0.5) continue;
                    double height = body.GetAltitude(hits[i].point);
                    if (height <= LastDeckASL) continue;
                    LastDeckASL = height;
                    LastQueryUsedDeck = true;
                }
                return LastDeckASL;
            }
            finally
            {
                // Hit buffers must not keep scene colliders alive after teardown.
                Array.Clear(hits, 0, count);
            }
        }

        private void FindKK()
        {
            if (!_lookedForKK)
            {
                _lookedForKK = true;
                foreach (Assembly assembly in AppDomain.CurrentDomain.GetAssemblies())
                    if (assembly.GetName().Name == "KerbalKonstructs")
                    {
                        // KK Activate adds this marker to its collider objects.
                        _kkColliderMarker = assembly.GetType("KerbalKonstructs.Core.KKMouseUtility", false);
                        break;
                    }
            }
        }
    }
}
