using System;

namespace MuMech
{
    // Three physics samples, unity DC gain, one-step delay, zero gain at Nyquist.
    // No deadband: slow real rotation, including roll, remains observable.
    public sealed class AttitudeRateFilter
    {
        private bool _ready;
        private double _previous, _older, _time, _step, _output;

        public void Reset() => _ready = false;

        public double Update(double rate, double time, double step)
        {
            if (double.IsNaN(rate) || double.IsInfinity(rate) ||
                double.IsNaN(time) || double.IsInfinity(time) ||
                double.IsNaN(step) || double.IsInfinity(step) || step <= 0)
            {
                Reset();
                return 0;
            }
            if (_ready && time == _time) return _output;
            if (!_ready || time < _time || time - _time > 1.5 * step ||
                Math.Abs(step - _step) > 1e-6 * step)
            {
                _previous = _older = rate;
                _output = rate;
                _ready = true;
            }
            else
            {
                _output = .25 * rate + .5 * _previous + .25 * _older;
                _older = _previous;
                _previous = rate;
            }
            _time = time;
            _step = step;
            return _output;
        }
    }
}
