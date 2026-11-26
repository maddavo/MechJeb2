using System;
using D=MuMech.Landing.AtmosphericLandingDynamics;

namespace MuMech.Landing
{
    // Independent radial screening and best-effort hazard response. These
    // are no-drag local bounds, NOT certified 3-D reachable sets or terrain
    // predictions. A warning alone never initiates premature braking.
    internal static class AtmosphericRecoverability
    {
        internal enum Level { Unknown, Clear, Warning, Hazard }
        internal readonly struct Assessment
        {
            internal readonly Level Status;
            internal readonly double DownSpeed,OptimisticDistance,DelayedDistance,Margin,FuelMargin;
            internal readonly string Reason;
            internal Assessment(Level status,string reason,double down=double.NaN,double optimistic=double.NaN,
                double delayed=double.NaN,double margin=double.NaN,double fuelMargin=double.NaN)
            {Status=status;Reason=reason;DownSpeed=down;OptimisticDistance=optimistic;DelayedDistance=delayed;
                Margin=margin;FuelMargin=fuelMargin;}
        }
        internal static Assessment Assess(D.State state,D.V surfaceVelocity,double clearance,double rawAcceleration,
            double cap,double gravity,double turnRate,double response,double maxFlow,double reserve,double touchdown)
        {
            if(!state.Valid||!surfaceVelocity.Valid||!D.Finite(clearance)||clearance<=0||
                !D.Finite(rawAcceleration)||rawAcceleration<0||!D.Finite(cap)||cap<=0||cap>1||
                !D.Finite(gravity)||gravity<0||!D.Finite(turnRate)||turnRate<0||
                !D.Finite(response)||response<0||!D.Finite(maxFlow)||maxFlow<0||
                !D.Finite(reserve)||reserve<=0||!D.Finite(touchdown)||touchdown<=0)
                return new Assessment(Level.Unknown,"invalid measured input or authority");
            double down=Math.Max(0,-D.V.Dot(surfaceVelocity,state.R.Unit));
            double free=state.Mass-reserve,up=rawAcceleration*cap-gravity;
            if(free<=0)return new Assessment(Level.Hazard,"protected reserve reached",down);
            // Current raw thrust, no drag or turn delay, full upward pointing.
            // A local no-drag screen may be pessimistic with beneficial drag
            // and optimistic with adverse lift, declining thrust, or slope.
            double optimistic=down<=touchdown?0:up>0?
                (down*down-touchdown*touchdown)/(2*up):double.PositiveInfinity;
            double angle=Math.Acos(Math.Max(-1,Math.Min(1,D.V.Dot(state.Forward,state.R.Unit))));
            double turn=angle<.01?0:turnRate>0?angle/turnRate:double.PositiveInfinity;
            double delay=turn+3*response+.1;
            double delayedSpeed=down+gravity*delay;
            double reduced=.9*rawAcceleration*cap-gravity;
            double brake=reduced>0?Math.Max(0,delayedSpeed-touchdown)/reduced:double.PositiveInfinity;
            double delayed=down*delay+.5*gravity*delay*delay+
                (reduced>0?Math.Max(0,delayedSpeed*delayedSpeed-touchdown*touchdown)/(2*reduced):double.PositiveInfinity);
            double fuel=free-(maxFlow>0?maxFlow*cap*(delay+brake):0);
            double margin=clearance-delayed;
            // No-drag stopping failure at high altitude is only a warning:
            // atmosphere, orbital curvature and future thrust may recover it.
            // Override only for a short local time-to-contact hazard.
            bool urgent=down>touchdown+.5&&clearance/Math.Max(.1,down)<=8;
            if(urgent&&down>touchdown+1&&(!D.Finite(optimistic)||optimistic>=clearance))
                return new Assessment(Level.Hazard,"immediate no-drag radial stopping bound exhausted",
                    down,optimistic,delayed,margin,fuel);
            if(urgent&&rawAcceleration*cap<=gravity&&down>touchdown+.5)
                return new Assessment(Level.Hazard,"insufficient upward authority",down,optimistic,delayed,margin,fuel);
            if(!D.Finite(delayed)||!D.Finite(optimistic)||optimistic>=clearance||margin<0||fuel<0)
                return new Assessment(Level.Warning,"delayed no-drag recoverability not demonstrated",
                    down,optimistic,delayed,margin,fuel);
            return new Assessment(Level.Clear,"local radial screen clear (not certified)",down,optimistic,delayed,margin,fuel);
        }

        // After plan loss, explicitly abandon target accuracy. Match a
        // clearance-dependent descent envelope, retain upward acceleration
        // priority, damp lateral velocity within a shrinking tilt envelope.
        // Never wait on a solver; never use the legacy fixed descent speed.
        internal static bool TryBestEffort(D.State state,D.V surfaceVelocity,double clearance,double rawAcceleration,
            double cap,double gravity,double response,double maxFlow,double reserve,double touchdown,
            bool immediate,out D.Command command)
        {
            command=default;
            if(!state.Valid||!surfaceVelocity.Valid||!D.Finite(clearance)||clearance<=0||
                !D.Finite(rawAcceleration)||rawAcceleration<=0||!D.Finite(cap)||cap<=0||cap>1||
                !D.Finite(gravity)||gravity<0||!D.Finite(response)||response<0||
                !D.Finite(maxFlow)||maxFlow<=0||!D.Finite(reserve)||reserve<=0||
                !D.Finite(touchdown)||touchdown<=0||state.Mass<=reserve)return false;
            // Bound the commanded flow over a short control interval plus
            // estimated engine tail. If the attained tail alone exhausts
            // the protected reserve, do not request additional thrust.
            double fuel=state.Mass-reserve,tail=maxFlow*state.Throttle*response;
            double fuelCap=Math.Max(0,(fuel-tail)/(maxFlow*(.2+response)));
            double allowed=Math.Min(cap,fuelCap);
            if(allowed<=0)return false;
            D.V up=state.R.Unit;
            double vertical=D.V.Dot(surfaceVelocity,up),down=Math.Max(0,-vertical);
            D.V horizontal=surfaceVelocity-up*vertical;
            double net=Math.Max(0,rawAcceleration*allowed-gravity);
            // No altitude-independent descent speed. At large clearance
            // allow speed to increase rather than burn merely to hover.
            double desired=Math.Sqrt(touchdown*touchdown+2*.45*net*clearance);
            double axial=immediate&&down>touchdown+1?rawAcceleration*allowed:
                Math.Max(0,gravity+.8*(-desired-vertical));
            axial=Math.Min(axial,rawAcceleration*allowed);
            D.V lateral=horizontal*(-.25);
            double tilt=Math.Min(Math.PI/6,Math.PI/60+clearance*Math.PI/1800);
            double maxLateral=Math.Min(axial*Math.Tan(tilt),
                Math.Sqrt(Math.Max(0,Math.Pow(rawAcceleration*allowed,2)-axial*axial)));
            if(lateral.Length>maxLateral)lateral=lateral.Unit*maxLateral;
            D.V force=up*axial+lateral;
            D.V direction=force.Length>1e-9?force.Unit:up;
            // Never energise while pointed substantially away from the
            // upward hemisphere. Actual attitude remains under MJ control.
            double throttle=D.V.Dot(state.Forward,up)>.5&&D.V.Dot(state.Forward,direction)>.75?
                Math.Min(allowed,force.Length/rawAcceleration):0;
            command=new D.Command(throttle,direction);
            return command.Valid;
        }
    }
}
