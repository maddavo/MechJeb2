namespace MuMech.AttitudeControllers
{
    public class TorquePI
    {
        private KosPIDLoop _loop { get; } = new KosPIDLoop();
        private double _responseGain;

        public double Update(double input, double setpoint, double momentOfInertia, double maxOutput)
        {
            _loop.Ki = 4 * momentOfInertia;
            _loop.Kp = 4 * momentOfInertia;
            return _loop.Update(input, setpoint, maxOutput);
        }

        public void ResetI()
        {
            _loop.ResetI();
            _responseGain = 0;
        }

        public double UpdateWithResponse(double input, double setpoint, double momentOfInertia,
            double maxOutput, double gain, double dt)
        {
            if (gain <= 0 || momentOfInertia <= 0 || maxOutput <= 0 || dt <= 0)
            {
                ResetI();
                return 0;
            }
            // Ki=Kp^2/(4*I): retain the original critically damped rate-loop
            // ratio. At gain=4 these are exactly the original Kp=Ki=4*I.
            if (_responseGain > gain)
            {
                double ratio = gain / _responseGain;
                _loop.ITerm *= ratio * ratio;
            }
            _responseGain = gain;
            _loop.Kp = gain * momentOfInertia;
            _loop.Ki = .25 * gain * gain * momentOfInertia;
            return _loop.UpdateWithDeltaTime(input, setpoint, maxOutput, dt);
        }
    }
}
