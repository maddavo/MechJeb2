using KSP.Localization;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class KillHorizontalVelocity : AutopilotStep
        {
            // Do not hand a low-gravity, high-TWR vessel to FinalDescent while it is
            // still translating materially sideways.  FinalDescent controls the
            // vertical approach; this step owns the hover-and-translation phase.
            private const double FinalDescentHorizontalSpeed = 1.0;

            public KillHorizontalVelocity(MechJebCore core) : base(core)
            {
            }

            // TODO I think that this function could be better rewritten to much more agressively kill the horizontal velocity. At present on low gravity bodies such as Bop, the craft will hover and slowly drift sideways, loosing the prescion of the landing. 
            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (!Core.Landing.PredictionReady)
                    return this;

                Vector3d horizontalPointingDirection = Vector3d.Exclude(VesselState.Up, VesselState.Forward).normalized;
                if (VesselState.SpeedSurfaceHorizontal <= FinalDescentHorizontalSpeed)
                {
                    Core.Thrust.RequestActiveThrottle(0.0f);
                    Core.Attitude.attitudeTo(Vector3.up, AttitudeReference.SURFACE_NORTH, Core.Landing);
                    return new FinalDescent(Core);
                }

                //control thrust to control vertical speed:
                const double DESIRED_SPEED = 0; // hold height until horizontal velocity is killed
                double controlledSpeed = Vector3d.Dot(VesselState.SurfaceVelocity, VesselState.Up);
                double verticalThrustAcceleration =
                    Vector3d.Dot(VesselState.Forward, VesselState.Up) * VesselState.MaxThrustAcceleration;
                double requestedThrottle = TerminalTranslationThrottlePolicy.ComputeThrottle(controlledSpeed,
                    DESIRED_SPEED, VesselState.LocalGravity, verticalThrustAcceleration);

                // The generic minimum-throttle setting is useful for ordinary
                // manoeuvre burns, but it must not floor terminal translation.
                // On Minmus a 5% floor can be more thrust than gravity and make
                // a craft climb while this step is trying to remove drift.
                Core.Thrust.RequestActiveThrottle((float)requestedThrottle, enforceMinimum: false, allowZero: true);

                //angle up and slightly away from vertical:
                Vector3d desiredThrustVector = (VesselState.Up + 0.2 * horizontalPointingDirection).normalized;

                Core.Attitude.attitudeTo(desiredThrustVector, AttitudeReference.INERTIAL, Core.Landing);

                Status = Localizer.Format("#MechJeb_LandingGuidance_Status10"); //"Killing horizontal velocity before final descent"

                return this;
            }
        }
    }
}
