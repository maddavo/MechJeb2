using System;

namespace MuMech.Landing
{
    /// <summary>
    /// Fixed body-relative target frame derived from the unpowered pass. Brake
    /// timing changes downrange; crossrange is reported without feeding it back
    /// into the one-dimensional brake-time search.
    /// </summary>
    internal readonly struct TargetAwareApproachFrame
    {
        private readonly Vector3d _target;
        private readonly Vector3d _downrange;
        private readonly Vector3d _crossrange;
        private readonly double _radius;

        private TargetAwareApproachFrame(Vector3d target, Vector3d downrange, double radius)
        {
            _target = target;
            _downrange = downrange;
            _crossrange = Vector3d.Cross(target, downrange).normalized;
            _radius = radius;
        }

        internal static bool TryCreate(Vector3d targetSurfaceDirection, Vector3d ballisticContactDirection,
            double bodyRadius, out TargetAwareApproachFrame frame)
        {
            frame = default(TargetAwareApproachFrame);
            if (!Finite(bodyRadius) || bodyRadius <= 0 || !Finite(targetSurfaceDirection) ||
                !Finite(ballisticContactDirection) || targetSurfaceDirection.sqrMagnitude < 1e-12 ||
                ballisticContactDirection.sqrMagnitude < 1e-12)
                return false;

            Vector3d target = targetSurfaceDirection.normalized;
            Vector3d towardBallistic = ballisticContactDirection.normalized;
            Vector3d tangent = towardBallistic - Vector3d.Dot(towardBallistic, target) * target;
            if (tangent.sqrMagnitude < 1e-12)
                return false;
            frame = new TargetAwareApproachFrame(target, tangent.normalized, bodyRadius);
            return true;
        }

        internal static bool TryCreateFromTrack(Vector3d targetSurfaceDirection,
            Vector3d referencePosition, Vector3d referenceVelocity,
            Vector3d angularVelocity, double bodyRadius, out TargetAwareApproachFrame frame)
        {
            frame = default(TargetAwareApproachFrame);
            if (!Finite(bodyRadius) || bodyRadius <= 0 ||
                !Finite(targetSurfaceDirection) || !Finite(referencePosition) ||
                !Finite(referenceVelocity) || !Finite(angularVelocity) ||
                targetSurfaceDirection.sqrMagnitude < 1e-12 ||
                referencePosition.sqrMagnitude < 1e-12)
                return false;

            Vector3d target = targetSurfaceDirection.normalized;
            Vector3d radial = referencePosition.normalized;
            Vector3d groundTrack = referenceVelocity - Vector3d.Cross(angularVelocity,
                referencePosition);
            groundTrack -= Vector3d.Dot(groundTrack, radial) * radial;
            if (groundTrack.sqrMagnitude < 1e-12) return false;

            // Parallel-transport the local direction of travel from the
            // orbital sample to the target tangent plane using their great
            // circle plane. This defines a search axis; it does not claim the
            // unpowered path lands at the target or solve crossrange.
            Vector3d planeNormal = Vector3d.Cross(radial, groundTrack).normalized;
            Vector3d downrange = Vector3d.Cross(planeNormal, target);
            downrange -= Vector3d.Dot(downrange, target) * target;
            if (downrange.sqrMagnitude < 1e-12) return false;
            frame = new TargetAwareApproachFrame(target, downrange.normalized, bodyRadius);
            return true;
        }

        internal bool TryProject(Vector3d endpointDirection, out double downrange, out double crossrange)
        {
            downrange = double.NaN;
            crossrange = double.NaN;
            if (!Finite(endpointDirection) || endpointDirection.sqrMagnitude < 1e-12)
                return false;
            Vector3d endpoint = endpointDirection.normalized;
            downrange = _radius * Math.Atan2(Vector3d.Dot(endpoint, _downrange),
                Vector3d.Dot(endpoint, _target));
            crossrange = _radius * Math.Asin(Math.Max(-1, Math.Min(1,
                Vector3d.Dot(endpoint, _crossrange))));
            return Finite(downrange) && Finite(crossrange);
        }

        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        private static bool Finite(Vector3d value) => Finite(value.x) && Finite(value.y) && Finite(value.z);
    }
}
