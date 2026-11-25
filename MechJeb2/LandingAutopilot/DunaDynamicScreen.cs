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
                double radius=D.Finite(terminalRadius)&&terminalRadius>0?terminalRadius:env.TerrainRadius+env.BottomOffset;
                // Reconstructing a vector and measuring its length can
                // round below the exact first pressure knot.
                if(!env.AirAtAltitude(radius-env.Radius,out double p,out _))
                {capture=false;reason="terminal_pressure_unknown";}
                else
                {
                    D.Authority authority=prop.At(p);
                    double gravity=env.Mu/(radius*radius);
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
        internal enum ProbeStatus { Feasible, Rejected, TerminalUnresolved, Unknown }
        internal static ProbeStatus Status(DunaBrakingHandoff.Result result)
        {
            if(result.Safe)return ProbeStatus.Feasible;
            switch(result.Reason)
            {
                case "ContactUnsafe":
                case "ReserveExhausted":
                case "terminal_clearance_exhausted":
                    return ProbeStatus.Rejected;
                case "terminal_authority_unresolved":
                case "terminal_authority_unknown":
                case "terminal_pressure_unknown":
                case "terminal_fuel_reserve":
                    return ProbeStatus.TerminalUnresolved;
                default:
                    return ProbeStatus.Unknown;
            }
        }
        internal static Decision Classify(DunaBrakingHandoff.Result delayed,DunaBrakingHandoff.Result immediate)
        {
            ProbeStatus later=Status(delayed),earlier=Status(immediate);
            if(later==ProbeStatus.Feasible)return Decision.Continue;
            // Only a demonstrated violation of the delayed candidate can
            // establish a tested deadline. Unknown is NOT rejected.
            if(later==ProbeStatus.Rejected&&earlier==ProbeStatus.Feasible)return Decision.BrakeDeadline;
            if(immediate.Reason=="ContactUnsafe"||immediate.Reason=="terminal_clearance_exhausted"||
                (earlier==ProbeStatus.TerminalUnresolved&&
                    (delayed.Reason=="ContactUnsafe"||delayed.Reason=="terminal_clearance_exhausted")))
                return Decision.Emergency;
            if(later==ProbeStatus.TerminalUnresolved||earlier==ProbeStatus.TerminalUnresolved)
                return Decision.TerminalUnresolved;
            return Decision.HoldUnresolved;
        }

        // Independent best-effort geometric near-contact fallback. Neither
        // an expired retry timer nor atmosphere entry is a braking deadline.
        internal static bool UnresolvedEmergency(double now,double terrainUT,double prep,double clearance,bool descending)
        {
            if(!descending||!D.Finite(now))return false;
            if(D.Finite(clearance)&&clearance<=1000)return true;
            return D.Finite(terrainUT)&&terrainUT>=now&&D.Finite(prep)&&prep>=0&&terrainUT-now<=prep;
        }

        internal const double TerminalClearance=300;
        internal static bool CanCaptureRecovery(double speed,double acceleration,double gravity,double clearance)
            =>D.Finite(speed)&&speed>=0&&speed<=CaptureSpeed&&D.Finite(acceleration)&&D.Finite(gravity)&&
                gravity>0&&acceleration>gravity*Math.Sqrt(1.04)&&D.Finite(clearance)&&clearance>=0;
        internal static bool CanTerminalHandoff(double speed,double acceleration,double gravity,double clearance)
            =>CanCaptureRecovery(speed,acceleration,gravity,clearance)&&clearance<=TerminalClearance;
        internal static bool CorrectionFits(double now,double aim,double duration,double entryCutoff,
            double tail,double brakingPrep,double delay)
            =>D.Finite(now)&&D.Finite(aim)&&aim>=0&&D.Finite(duration)&&duration>0&&D.Finite(tail)&&tail>=0&&
                D.Finite(brakingPrep)&&brakingPrep>=0&&D.Finite(delay)&&
                DunaBrakingHandoff.EntryFits(now,aim+duration,entryCutoff)&&
                aim+duration+tail+brakingPrep<=delay+1e-8;
    }
}
