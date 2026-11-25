using System;

namespace MuMech.Landing
{
    // Passive timing observations. This never requests an actuator or a warp.
    internal sealed class V1WarpPreparation
    {
        private bool _observing;
        private double _elapsed, _turnEnd = double.NaN, _angle;
        private Vector3d _command;
        private bool _hasDirectionSample;
        private bool _lastPhysicalReady;
        private Vector3d _previousForward, _previousCommand;
        internal bool IsObserving => _observing;
        internal double ResidualAngularSpeed { get; private set; } = double.PositiveInfinity;
        internal bool Ready(double error) => V1LandingControlPolicy.BrakeWarpReady(error, ResidualAngularSpeed);
        internal bool ReadyForWarp(double error, bool physicsRunning, bool ownsRequest) =>
            error < 5 && (physicsRunning ? Ready(error) : ownsRequest && _lastPhysicalReady);
        internal double MeasuredAngle { get; private set; }
        internal double TurnSeconds { get; private set; }
        internal double SettleSeconds { get; private set; }
        internal double WarpOverrunSeconds { get; private set; }
        internal double ActualWarpExitUT { get; private set; } = double.NaN;
        private bool _wasWarping;
        private double _requestedExit = double.NaN;
        internal double KnownPreparationSeconds { get; private set; } = double.NaN;
        internal void RetainMeasuredPreparation(double seconds)
        {
            if (Finite(seconds) && seconds >= 0) KnownPreparationSeconds = seconds;
        }

        internal void Observe(Vector3d forward, Vector3d command, double angularSpeed,
            double dt, bool physicsRunning)
        {
            if (!physicsRunning)
            {
                _observing = false; _hasDirectionSample = false;
                ResidualAngularSpeed = double.PositiveInfinity;
                return;
            }
            if (forward.sqrMagnitude == 0 || command.sqrMagnitude == 0 ||
                !Finite(dt) || dt <= 0 || !Finite(angularSpeed) || angularSpeed < 0)
            {
                _hasDirectionSample = false; _lastPhysicalReady = false;
                ResidualAngularSpeed = double.PositiveInfinity;
                return;
            }
            forward = forward.normalized; command = command.normalized;
            ResidualAngularSpeed = angularSpeed;
            if (_hasDirectionSample)
            {
                Vector3d heading = DirectionRate(_previousForward, forward, dt);
                Vector3d tracking = DirectionRate(_previousCommand, command, dt);
                // Heading directions share the inertial frame. The scalar
                // vessel rate retains spin not explained by heading motion,
                // including roll; matching command motion cannot hide it.
                double spinSquared = Math.Max(0, angularSpeed * angularSpeed - heading.sqrMagnitude);
                ResidualAngularSpeed = Math.Sqrt((heading - tracking).sqrMagnitude + spinSquared);
                if (!Finite(heading.magnitude) || heading.magnitude > angularSpeed + 0.001)
                    ResidualAngularSpeed = double.PositiveInfinity;
            }
            _previousForward = forward; _previousCommand = command; _hasDirectionSample = true;
            double error = Vector3d.Angle(forward, command);
            _lastPhysicalReady = Ready(error);
            if (_observing && Vector3d.Angle(_command, command) >= 5) _observing = false;
            if (!_observing && error >= 5)
            {
                _observing = true; _elapsed = 0; _angle = error;
                _turnEnd = double.NaN; _command = command;
            }
            if (!_observing) return;
            _elapsed += dt;
            if (error < 5 && double.IsNaN(_turnEnd)) _turnEnd = _elapsed;
            if (!Ready(error)) return;
            MeasuredAngle = _angle;
            TurnSeconds = _turnEnd;
            SettleSeconds = Math.Max(dt, _elapsed - _turnEnd);
            _observing = false;
        }

        private static Vector3d DirectionRate(Vector3d before, Vector3d after, double dt)
        {
            Vector3d axis = Vector3d.Cross(before, after);
            double sine = axis.magnitude;
            double angle = Math.Atan2(sine, Math.Max(-1, Math.Min(1, Vector3d.Dot(before, after))));
            // A direction reversal has no uniquely observed rotation axis.
            // Reject readiness rather than treating that jump as stationary.
            if (sine < 1e-12) return angle > 1 ? new Vector3d(double.PositiveInfinity, 0, 0) : Vector3d.zero;
            return axis * (angle / (sine * dt));
        }

        internal double Estimate(double requiredAngle, double fallback = double.NaN)
        {
            if (!Finite(requiredAngle) || requiredAngle < 0) return fallback;
            if (MeasuredAngle <= 0 || !Finite(TurnSeconds))
                return Finite(fallback) ? fallback : KnownPreparationSeconds;
            // Square-root scaling covers acceleration-limited small turns;
            // linear scaling covers larger turns with an observed rate limit.
            double ratio = requiredAngle / MeasuredAngle;
            return TurnSeconds * Math.Max(ratio, Math.Sqrt(ratio)) + SettleSeconds;
        }

        internal void ObserveWarp(double now, double rate)
        {
            if (_wasWarping && rate <= 1)
            {
                ActualWarpExitUT = now;
                if (Finite(_requestedExit))
                    WarpOverrunSeconds = Math.Max(WarpOverrunSeconds, Math.Max(0, now - _requestedExit));
            }
            _wasWarping = rate > 1;
        }

        internal void RequestedWarpExit(double ut) => _requestedExit = ut;
        internal bool OwnsWarpTarget(double ut) => Finite(_requestedExit) && ut == _requestedExit;
        internal void ClearWarpRequest() => _requestedExit = double.NaN;

        internal double ExitUT(double eventUT, double now, double turnAngle,
            double fallback, double physicsTick)
        {
            double preparation = Estimate(turnAngle, fallback);
            double exit = eventUT - preparation - WarpOverrunSeconds - physicsTick;
            return Finite(preparation) && preparation >= 0 && Finite(exit) && exit > now ? exit : double.NaN;
        }

        internal static bool UnpoweredWarpAllowed(bool autoWarp, bool airlessTargeted,
            bool landableFresh, bool poweredObserved, bool attitudeHeld, bool ascending,
            double commandedThrottle, double appliedThrottle, double thrustAcceleration,
            double minimumThrottle, double minimumThrust, double now, double firstThrustUT,
            double contactUT, double exitUT) => autoWarp && airlessTargeted && landableFresh &&
            // Ascending is a deterministic zero-throttle coast in V1, not an
            // attitude failure. The fresh terrain-validated forecast must still
            // supply a later first-thrust deadline and a preparation-safe exit.
            !poweredObserved && !attitudeHeld &&
            commandedThrottle == 0 && appliedThrottle == 0 && thrustAcceleration == 0 &&
            minimumThrottle == 0 && minimumThrust == 0 && Finite(firstThrustUT) &&
            Finite(contactUT) && Finite(exitUT) && now < exitUT && exitUT < firstThrustUT &&
            firstThrustUT < contactUT;

        private static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
    }
}
