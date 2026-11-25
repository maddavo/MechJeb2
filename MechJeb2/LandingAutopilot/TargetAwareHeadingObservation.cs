using System;

namespace MuMech.Landing
{
    // Passive world-frame heading motion; excludes roll and avoids mixing
    // KSP's vessel-local angular-velocity convention with a world turn axis.
    internal sealed class TargetAwareHeadingObservation
    {
        private object _body;
        private Vector3d _forward;
        private double _ut = double.NaN;
        internal Vector3d Rate { get; private set; }

        internal void Observe(object body, Vector3d forward, double ut, double dt, bool physics)
        {
            Rate = Vector3d.zero;
            double elapsed = ut - _ut;
            if (physics && ReferenceEquals(body, _body) && elapsed > 0 && elapsed <= 2 * dt &&
                forward.sqrMagnitude > 0 && _forward.sqrMagnitude > 0)
            {
                Vector3d axis = Vector3d.Cross(_forward, forward.normalized);
                double sine = axis.magnitude;
                if (sine > 1e-12)
                    Rate = axis * (Math.Atan2(sine,
                        Vector3d.Dot(_forward, forward.normalized)) / (sine * elapsed));
            }
            _body = physics ? body : null;
            _forward = forward.normalized;
            _ut = ut;
        }
    }
}
