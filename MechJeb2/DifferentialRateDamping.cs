using System;

namespace MuMech
{
    // Supply only rate-opposing torque when the ordinary command is already
    // trying to brake the rotation and the measured rate is not settling.
    internal sealed class DifferentialRateDamping
    {
        private double _previousRate, _engageTime, _releaseTime;
        private bool _hasRate, _active;

        internal void Reset()
        {
            _hasRate = _active = false;
            _engageTime = _releaseTime = 0;
        }

        internal double Update(double rate, double ordinaryCommand, double inertia, double headroom, double dt)
        {
            if (!(dt > 0) || !(inertia > 0) || !(headroom > 0) || double.IsNaN(rate) || double.IsInfinity(rate))
            {
                Reset();
                return 0;
            }

            double previous = _previousRate;
            _previousRate = rate;
            bool ready = _hasRate;
            _hasRate = true;
            if (!ready || ordinaryCommand * rate >= 0 || previous * rate <= 0)
            {
                _active = false;
                _engageTime = _releaseTime = 0;
                return 0;
            }

            double speed = Math.Abs(rate);
            double deceleration = (Math.Abs(previous) - speed) / dt;
            _engageTime = speed > .006 && deceleration < 2 * speed ? _engageTime + dt : 0;
            _releaseTime = speed < .003 || deceleration > speed / .3 ? _releaseTime + dt : 0;
            if (_engageTime >= .06) _active = true;
            if (_releaseTime >= .16) _active = false;
            // Limit the added contribution to the torque needed for a half-second
            // stop, with a fractional headroom cap for the auxiliary damper.
            return _active ? -Math.Sign(rate) * Math.Min(headroom * .30, inertia * speed * 2) : 0;
        }
    }
}
