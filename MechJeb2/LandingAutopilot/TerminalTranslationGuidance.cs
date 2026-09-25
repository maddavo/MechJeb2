using System;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        /// <summary>
        /// Forms a bounded thrust direction that holds altitude while removing
        /// horizontal velocity. The lateral component always opposes actual
        /// surface velocity; it never derives from the vessel's present tilt.
        /// </summary>
        public static class TerminalTranslationGuidance
        {
            private const double HorizontalBrakingTimeConstant = 2.0;
            private const double MaximumHorizontalToVerticalRatio = 0.70;

            public static Vector3d DesiredThrustDirection(Vector3d up, Vector3d surfaceVelocity,
                double localGravity)
            {
                if (!Finite(up) || !Finite(surfaceVelocity) || !Finite(localGravity) ||
                    localGravity <= 0 || up.sqrMagnitude <= 0)
                    return Vector3d.up;

                Vector3d unitUp = up.normalized;
                Vector3d horizontalVelocity = Vector3d.Exclude(unitUp, surfaceVelocity);
                double horizontalSpeed = horizontalVelocity.magnitude;
                if (horizontalSpeed <= 1e-6)
                    return unitUp;

                // Seek a velocity decay with a two-second time constant, but
                // cap the tilt at about 35 degrees so vertical control remains
                // authoritative on low-gravity bodies.
                double horizontalAcceleration = Math.Min(horizontalSpeed / HorizontalBrakingTimeConstant,
                    localGravity * MaximumHorizontalToVerticalRatio);
                return (unitUp - horizontalVelocity.normalized * (horizontalAcceleration / localGravity)).normalized;
            }

            private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
            private static bool Finite(Vector3d value) => Finite(value.x) && Finite(value.y) && Finite(value.z);
        }
    }
}