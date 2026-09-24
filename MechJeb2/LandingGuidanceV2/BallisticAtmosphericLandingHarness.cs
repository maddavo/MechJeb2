using System;

namespace MuMech
{
    /// <summary>
    /// Deterministic one-dimensional validation for V2's ballistic atmospheric
    /// descent assumption. It models a stable retrograde capsule/bell profile:
    /// either parachutes reduce descent speed or V2 enters powered braking,
    /// including a staged recovery from a failed powered stage. It owns no KSP
    /// objects and is a controller gate, not a claim to replace KSP aerophysics.
    /// </summary>
    public sealed class BallisticAtmosphericLandingHarness
    {
        public BallisticAtmosphericLandingHarnessResult Execute(double initialAltitude, double initialDownSpeed,
            double gravity, double initialMaximumAcceleration, bool parachutesDeploy, bool injectStageFailure,
            double physicsStepSeconds = 0.02)
        {
            var result = new BallisticAtmosphericLandingHarnessResult();
            if (!Finite(initialAltitude) || !Finite(initialDownSpeed) || !Finite(gravity) ||
                !Finite(initialMaximumAcceleration) || initialAltitude <= 0 || initialDownSpeed < 0 ||
                gravity <= 0 || initialMaximumAcceleration <= gravity)
            {
                result.RejectionReason = "The ballistic descent harness requires a positive altitude, descent speed, gravity, and thrust above gravity.";
                return result;
            }

            double altitude = initialAltitude;
            double downSpeed = initialDownSpeed;
            double maximumAcceleration = initialMaximumAcceleration;
            double dt = Math.Max(0.005, Math.Min(0.10, physicsStepSeconds));
            double ut = 0;
            double poweredSeconds = 0;
            bool stageFailureActive = false;
            var stagingGate = new V2PoweredStagingGate();

            for (int step = 0; step < 200000 && altitude > 0; ++step)
            {
                double throttle = 0;
                double measuredAcceleration = 0;
                if (parachutesDeploy)
                {
                    result.ParachutesDeployed = true;
                    // Stable ballistic craft with a deployed parachute trends
                    // continuously toward a conservative six metre/second
                    // descent without any powered command.
                    downSpeed += (6.0 - downSpeed) * Math.Min(1.0, 1.50 * dt);
                }
                else
                {
                    bool braking = AtmosphericEnergyGate.ShouldBeginPoweredBraking(altitude, downSpeed, gravity,
                        maximumAcceleration);
                    if (braking)
                    {
                        result.PoweredBrakingEntered = true;
                        AtmosphericBallisticBrakingCommand brakingCommand = AtmosphericBallisticBraking.Calculate(
                            altitude, downSpeed, gravity, 0, maximumAcceleration);
                        if (!brakingCommand.Valid)
                        {
                            result.RejectionReason = brakingCommand.RejectionReason;
                            break;
                        }
                        throttle = brakingCommand.RequestedThrottle;
                        poweredSeconds += dt;
                        if (injectStageFailure && poweredSeconds >= 0.10 && !result.StageCommanded)
                        {
                            stageFailureActive = true;
                            measuredAcceleration = 0;
                        }
                        else
                            measuredAcceleration = maximumAcceleration * throttle;

                        V2PoweredStageDecision stage = stagingGate.Decide(ut, true, false, throttle,
                            measuredAcceleration, maximumAcceleration, injectStageFailure && stageFailureActive);
                        if (stage.Directive == V2PoweredStageDirective.CommandStage)
                        {
                            result.StageCommanded = true;
                            stageFailureActive = false;
                            maximumAcceleration = initialMaximumAcceleration * 1.20;
                            measuredAcceleration = 0;
                            stagingGate.Reset();
                        }
                        if (stageFailureActive && !result.StageCommanded)
                            result.StageLossObserved = true;
                    }
                }

                result.MaximumThrottle = Math.Max(result.MaximumThrottle, throttle);
                result.MinimumThrottleWhenPowered = throttle > 0
                    ? Math.Min(result.MinimumThrottleWhenPowered, throttle) : result.MinimumThrottleWhenPowered;
                // Positive down speed: gravity increases it, retrograde thrust
                // reduces it. A stable vehicle cannot climb during landing.
                if (!parachutesDeploy)
                    downSpeed = Math.Max(0, downSpeed + (gravity - measuredAcceleration) * dt);
                altitude -= downSpeed * dt;
                ut += dt;
            }

            result.ElapsedSeconds = ut;
            result.TouchdownSpeed = downSpeed;
            result.Landed = altitude <= 0 && downSpeed <= 8.0;
            if (!result.Landed && result.RejectionReason == null)
                result.RejectionReason = "The ballistic descent controller did not achieve the eight metre/second touchdown bound; speed=" +
                    downSpeed.ToString("F2") + " altitude=" + altitude.ToString("F2") + ".";
            return result;
        }

        private static double Clamp01(double value) => !Finite(value) ? 0 : Math.Max(0, Math.Min(1, value));
        private static bool Finite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
    }

    public sealed class BallisticAtmosphericLandingHarnessResult
    {
        public bool Landed;
        public bool ParachutesDeployed;
        public bool PoweredBrakingEntered;
        public bool StageLossObserved;
        public bool StageCommanded;
        public double TouchdownSpeed;
        public double ElapsedSeconds;
        public double MaximumThrottle;
        public double MinimumThrottleWhenPowered = double.PositiveInfinity;
        public string RejectionReason;
    }
}
