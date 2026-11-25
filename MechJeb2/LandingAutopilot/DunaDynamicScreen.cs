using System;
using D=MuMech.Landing.AtmosphericLandingDynamics;

namespace MuMech.Landing
{
    // Experimental Duna-only screen. Capture is not touchdown.
    internal static class DunaDynamicScreen
    {
        internal const double Margin=.8, FuelReserveFraction=.10, CaptureClearance=300, CaptureSpeed=15;
        internal enum Decision { Continue, BrakeDeadline, HoldUnresolved, TerminalUnresolved, Emergency }
        internal static DunaBrakingHandoff.Result Probe(D.Environment env,D.Propulsion prop,D.Limits limits,D.State state,
            double delay,double correctionLead=0,double correctionSeconds=0,D.Command correction=default,double terminalRadius=double.NaN)
        {
            if(correctionSeconds==0) correction=new D.Command(0,state.Forward);
            D.Result r=D.Screen(env,prop,limits,state,correctionSeconds,correction,delay,1200,
                CaptureClearance,CaptureSpeed,correctionLead:correctionLead);
            double clearance=env.Clearance(r.End.R),speed=env.Surface(r.End.R,r.End.Velocity).Length;
            bool capture=r.Status==D.Outcome.CaptureEnvelope;
            string reason=r.Status.ToString();
            if(capture)
            {
                // Evaluate terminal authority AFTER propagating mass loss and
                // future pressure. Not a complete terminal control validation.
                var terminalR=r.End.R.Unit*(D.Finite(terminalRadius)&&terminalRadius>0?terminalRadius:env.TerrainRadius+env.BottomOffset);
                if(!env.Air(terminalR,out double p,out _))
                {capture=false;reason="terminal_pressure_unknown";}
                else
                {
                    D.Authority authority=prop.At(p);
                    double gravity=env.Mu/(terminalR.Length*terminalR.Length);
                    if(!authority.Known){capture=false;reason="terminal_authority_unknown";}
                    else if(!(authority.Force/r.End.Mass>gravity*Math.Sqrt(1.04)))
                    {capture=false;reason="terminal_authority_unresolved";}
                    else if(!(r.End.Mass-limits.ReserveMass>authority.Flow*10))
                    {capture=false;reason="terminal_fuel_reserve";}
                    else reason="dynamic_capture_envelope";
                }
            }
            return new DunaBrakingHandoff.Result(capture,reason,r.MinimumClearance,clearance,speed,r.End.UT-state.UT);
        }
        internal static Decision Classify(DunaBrakingHandoff.Result delayed,DunaBrakingHandoff.Result immediate)
        {
            if(delayed.Safe)return Decision.Continue;
            if(immediate.Safe)return Decision.BrakeDeadline;
            bool TerminalOnly(DunaBrakingHandoff.Result result) =>
                result.Reason=="terminal_authority_unresolved" || result.Reason=="terminal_authority_unknown" ||
                result.Reason=="terminal_pressure_unknown" || result.Reason=="terminal_fuel_reserve";
            if(TerminalOnly(delayed)&&TerminalOnly(immediate))return Decision.TerminalUnresolved;
            if(TerminalOnly(immediate)&&(delayed.Reason=="ContactUnsafe"||delayed.Reason=="ContactWithinLimits"||
                delayed.Reason=="ReserveExhausted"||delayed.Reason=="terminal_clearance_exhausted"))
                return Decision.Emergency;
            // Both failed: no demonstrated latest-safe ignition. A projected
            // collision/clearance or resource exhaustion under immediate
            // braking is a best-effort emergency, not a validated safe plan.
            if(immediate.Reason=="ContactUnsafe" || immediate.Reason=="ContactWithinLimits" ||
               immediate.Reason=="ReserveExhausted" || immediate.Reason=="terminal_clearance_exhausted")
                return Decision.Emergency;
            return Decision.HoldUnresolved;
        }
        internal static bool CanTerminalHandoff(double speed,double acceleration,double gravity,double clearance)
            =>D.Finite(speed)&&speed>=0&&speed<=CaptureSpeed&&D.Finite(acceleration)&&D.Finite(gravity)&&
                gravity>0&&acceleration>gravity*Math.Sqrt(1.04)&&D.Finite(clearance)&&clearance>=0;
        internal static bool CorrectionFits(double now,double aim,double duration,double entryCutoff,
            double tail,double brakingPrep,double delay)
            =>D.Finite(now)&&D.Finite(aim)&&aim>=0&&D.Finite(duration)&&duration>0&&D.Finite(tail)&&tail>=0&&
                D.Finite(brakingPrep)&&brakingPrep>=0&&D.Finite(delay)&&
                DunaBrakingHandoff.EntryFits(now,aim+duration,entryCutoff)&&
                aim+duration+tail+brakingPrep<=delay+1e-8;
    }
}
