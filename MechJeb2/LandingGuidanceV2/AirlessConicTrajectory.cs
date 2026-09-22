using System;
using MechJebLib.Primitives;
using MechJebLib.TwoBody;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Immutable body-centred two-body trajectory.  It deliberately has no
    /// Unity or vessel dependency, allowing a recorded guidance snapshot to be
    /// replayed by the same airless estimator and planner used in flight.
    /// </summary>
    public sealed class AirlessConicTrajectory
    {
        private readonly double _mu;
        private readonly double _epoch;
        private readonly Vector3d _position;
        private readonly Vector3d _velocity;

        public readonly double PeriapsisRadius;
        public readonly double Period;
        public bool IsBound => IsFinite(Period) && Period > 0;

        public AirlessConicTrajectory(double mu, double epoch, Vector3d position, Vector3d velocity)
        {
            _mu = mu;
            _epoch = epoch;
            _position = position;
            _velocity = velocity;

            double radius = position.magnitude;
            Vector3d angularMomentum = Vector3d.Cross(position, velocity);
            Vector3d eccentricityVector = Vector3d.Cross(velocity, angularMomentum) / mu - position / radius;
            double eccentricity = eccentricityVector.magnitude;
            double energy = velocity.sqrMagnitude / 2.0 - mu / radius;
            double semiMajorAxis = -mu / (2.0 * energy);
            PeriapsisRadius = semiMajorAxis * (1.0 - eccentricity);
            Period = energy < 0 && semiMajorAxis > 0
                ? 2.0 * Math.PI * Math.Sqrt(semiMajorAxis * semiMajorAxis * semiMajorAxis / mu)
                : double.NaN;
        }

        public bool TryStateAt(double ut, out Vector3d position, out Vector3d velocity)
        {
            position = Vector3d.zero;
            velocity = Vector3d.zero;
            if (!IsFinite(_mu) || _mu <= 0 || !IsFinite(ut) || ut < _epoch ||
                !IsFinite(_position) || !IsFinite(_velocity))
                return false;
            try
            {
                (V3 propagatedPosition, V3 propagatedVelocity) = Shepperd.Solve(_mu, ut - _epoch,
                    _position.ToV3(), _velocity.ToV3());
                position = propagatedPosition.ToVector3d();
                velocity = propagatedVelocity.ToVector3d();
                return IsFinite(position) && IsFinite(velocity);
            }
            catch (Exception)
            {
                return false;
            }
        }

        public bool TryNextRadiusCrossing(double fromUT, double radius, out double crossingUT)
        {
            crossingUT = double.NaN;
            if (!IsBound || !IsFinite(radius) || radius <= 0 || PeriapsisRadius > radius || fromUT < _epoch)
                return false;

            if (!TryStateAt(fromUT, out Vector3d firstPosition, out _)) return false;
            double previousUT = fromUT;
            double previousResidual = firstPosition.magnitude - radius;
            double step = Math.Max(0.25, Period / 128.0);
            double endUT = fromUT + Period;
            for (double ut = fromUT + step; ut <= endUT + 0.001; ut += step)
            {
                double sampleUT = Math.Min(ut, endUT);
                if (!TryStateAt(sampleUT, out Vector3d position, out _)) return false;
                double residual = position.magnitude - radius;
                if (previousResidual > 0 && residual <= 0)
                {
                    double low = previousUT;
                    double high = sampleUT;
                    for (int i = 0; i < 48; ++i)
                    {
                        double middle = (low + high) / 2.0;
                        if (!TryStateAt(middle, out Vector3d middlePosition, out _)) return false;
                        if (middlePosition.magnitude > radius) low = middle;
                        else high = middle;
                    }
                    crossingUT = (low + high) / 2.0;
                    return true;
                }
                previousUT = sampleUT;
                previousResidual = residual;
                if (sampleUT >= endUT) break;
            }
            return false;
        }

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        private static bool IsFinite(Vector3d value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
    }
}
