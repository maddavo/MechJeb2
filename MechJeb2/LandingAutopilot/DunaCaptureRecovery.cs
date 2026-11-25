namespace MuMech.Landing
{
    // Non-targeted recovery for an already captured trajectory above the
    // terminal region. Reuse existing controlled descent; do not claim
    // recovered horizontal range or a validated terminal trajectory.
    public sealed class DunaCaptureRecovery : FinalDescent
    {
        public DunaCaptureRecovery(MechJebCore core) : base(core) { }

        public override AutopilotStep Drive(FlightCtrlState s)
        {
            if (!Vessel.LandedOrSplashed && Core.Landing.DunaDynamicTerminalReady)
            {
                Core.Thrust.RequestActiveThrottle(0);
                return new FinalDescent(Core);
            }
            AutopilotStep next=base.Drive(s);
            if (next==this) Status="Duna capture recovery (not targeted terminal descent): "+
                VesselState.AltitudeBottom.ToString("F0")+" m";
            return next;
        }
    }
}
