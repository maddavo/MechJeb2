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
