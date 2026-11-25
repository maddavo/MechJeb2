using System;

namespace MuMech.Landing
{
    // Independent, bounded, no-aerodynamic-credit screening model. This is not
    // a landing forecast, optimal ignition search, or proof of feasibility.
    // No dependency on ordinary predictor publication, endpoint, speed-policy
    // domain or integration-start UT.
    internal static class DunaBrakingHandoff
    {
        internal const double AuthorityFraction = 0.8;
        internal const double TerminalReserve = 300;
        internal const double CaptureSpeed = 15;
        internal const double RefreshSeconds = 0.5;
        internal const double MaximumHorizon = 1200;
        internal const double GuardSeconds = 3;

        internal readonly struct Vec
        {
            internal readonly double X, Y, Z;
            internal Vec(double x, double y, double z) { X=x; Y=y; Z=z; }
            internal double Norm => Math.Sqrt(Dot(this,this));
            internal bool Valid => Finite(X) && Finite(Y) && Finite(Z);
            public static Vec operator +(Vec a, Vec b) => new Vec(a.X+b.X,a.Y+b.Y,a.Z+b.Z);
            public static Vec operator -(Vec a, Vec b) => new Vec(a.X-b.X,a.Y-b.Y,a.Z-b.Z);
            public static Vec operator *(Vec a, double k) => new Vec(a.X*k,a.Y*k,a.Z*k);
            internal static double Dot(Vec a, Vec b) => a.X*b.X+a.Y*b.Y+a.Z*b.Z;
            internal static Vec Cross(Vec a, Vec b) => new Vec(a.Y*b.Z-a.Z*b.Y,a.Z*b.X-a.X*b.Z,a.X*b.Y-a.Y*b.X);
        }

        internal readonly struct Result
        {
            internal readonly bool Safe;
            internal readonly string Reason;
            internal readonly double MinimumClearance, EndClearance, EndSpeed, Elapsed;
            internal Result(bool safe, string reason, double minimumClearance, double endClearance,
                double endSpeed, double elapsed)
            {
                Safe=safe; Reason=reason; MinimumClearance=minimumClearance;
                EndClearance=endClearance; EndSpeed=endSpeed; Elapsed=elapsed;
            }
        }

        // Terrain radius includes global upper terrain bound and current craft
        // bottom offset. Fixed derated acceleration; no mass-loss credit.
        // Airflow co-rotates, as in the ordinary atmospheric predictor.
        // No drag/lift credit; adverse lift, changing capability and actual
        // terminal dynamics remain uncertainty, not a certified safety bound.
        internal static Result Probe(Vec position, Vec orbitalVelocity, Vec omega, double mu,
            double terrainRadius, double availableAcceleration, double coastSeconds)
        {
            if (!position.Valid || !orbitalVelocity.Valid || !omega.Valid ||
                !Finite(mu) || mu<=0 || !Finite(terrainRadius) || terrainRadius<=0 ||
                !Finite(availableAcceleration) || availableAcceleration<=0 ||
                !Finite(coastSeconds) || coastSeconds<0 || coastSeconds>=MaximumHorizon)
                return new Result(false,"invalid_input",double.NaN,double.NaN,double.NaN,0);
            double initialClearance=position.Norm-terrainRadius;
            double initialSpeed=(orbitalVelocity-Vec.Cross(omega,position)).Norm;
            double acceleration=availableAcceleration*AuthorityFraction;
            if (!(acceleration > mu/(terrainRadius*terrainRadius)*Math.Sqrt(1.04)))
                return new Result(false,"insufficient_hover_authority",initialClearance,initialClearance,initialSpeed,0);

            Vec x=position, v=orbitalVelocity;
            double elapsed=0, minimum=initialClearance;
            for (int i=0; i<5000 && elapsed<MaximumHorizon; i++)
            {
                double r=x.Norm;
                Vec surface=v-Vec.Cross(omega,x);
                double speed=surface.Norm, clearance=r-terrainRadius;
                minimum=Math.Min(minimum,clearance);
                if (!Finite(r) || r<=0 || !Finite(speed) || !Finite(clearance))
                    return new Result(false,"nonfinite_projection",minimum,clearance,speed,elapsed);
                if (clearance<TerminalReserve)
                    return new Result(false,"terminal_clearance_exhausted",minimum,clearance,speed,elapsed);
                bool powered=elapsed>=coastSeconds;
                if (powered && speed<=CaptureSpeed)
                    return new Result(true,"capture_envelope",minimum,clearance,speed,elapsed);

                double dt=Math.Min(1,MaximumHorizon-elapsed);
                if (!powered) dt=Math.Min(dt,coastSeconds-elapsed);
                else if (speed<4*acceleration) dt=Math.Min(dt,0.25);
                if (!(dt>0)) break;

                Vec a0=Acceleration(x,v,omega,mu,acceleration,powered);
                Vec midX=x+v*(0.5*dt), midV=v+a0*(0.5*dt);
                Vec aMid=Acceleration(midX,midV,omega,mu,acceleration,powered);
                Vec nextX=x+midV*dt, nextV=v+aMid*dt;
                // Screen the straight segment's radial minimum as well as
                // midpoint; the model is not a terrain-contact validator.
                Vec chord=nextX-x;
                double chordSquared=Vec.Dot(chord,chord);
                double fraction=chordSquared>0?Math.Max(0,Math.Min(1,-Vec.Dot(x,chord)/chordSquared)):0;
                minimum=Math.Min(minimum,Math.Min(midX.Norm,(x+chord*fraction).Norm)-terrainRadius);
                elapsed+=dt; x=nextX; v=nextV;
                if (!Finite(minimum) || minimum<TerminalReserve)
                    return new Result(false,"terminal_clearance_exhausted",minimum,x.Norm-terrainRadius,
                        (v-Vec.Cross(omega,x)).Norm,elapsed);
            }
            return new Result(false,"projection_horizon",minimum,x.Norm-terrainRadius,
                (v-Vec.Cross(omega,x)).Norm,elapsed);
        }

        private static Vec Acceleration(Vec x, Vec v, Vec omega, double mu, double thrust, bool powered)
        {
            double r=x.Norm;
            Vec gravity=x*(-mu/(r*r*r));
            Vec surface=v-Vec.Cross(omega,x);
            double speed=surface.Norm;
            // Match the live controller's no-thrust-on-ascent gate.
            return powered && speed>1e-6 && Vec.Dot(surface,x)<=0
                ? gravity+surface*(-thrust/speed) : gravity;
        }

        // Reuse airless preparation measurements. Unmeasured fallback is a
        // conservative test parameter, not a measured worst-case turn bound.
        internal static double TurnReserve(double angle, double measured)
        {
            if (!Finite(angle) || angle<0 || angle>180) return double.NaN;
            double fallback=Math.Max(5,30*Math.Sqrt(angle/180));
            return Finite(measured) && measured>=0 ? Math.Max(fallback,1.5*measured) : fallback;
        }

        internal static double PreparationReserve(double turn, double engineResponse,
            double smoothing, double warpOverrun, double tick)
        {
            if (!Finite(turn) || turn<0 || !Finite(engineResponse) || engineResponse<0 ||
                !Finite(smoothing) || smoothing<0 || !Finite(warpOverrun) || warpOverrun<0 ||
                !Finite(tick) || tick<=0) return double.NaN;
            return turn+2*engineResponse+smoothing+warpOverrun+GuardSeconds+RefreshSeconds+tick;
        }

        // Independent entry deadline: no landing forecast is required.
        internal static double EntryCutoff(double entryUT, double turn, double tail, double margin)
        {
            if (!Finite(entryUT) || !Finite(turn) || !Finite(tail) || !Finite(margin) ||
                turn<0 || tail<0 || margin<0) return double.NaN;
            return entryUT-turn-tail-margin;
        }

        internal static bool EntryFits(double now, double duration, double cutoff) =>
            Finite(now) && Finite(duration) && duration>=0 && Finite(cutoff) && now+duration<cutoff;

        // Search only bounded, independently screened correction commands.
        internal static double SelectPulse(double requested, double cap, Func<double,bool> feasible)
        {
            if (!Finite(requested) || !Finite(cap) || requested<=0 || cap<=0 || feasible==null) return 0;
            double dv=Math.Min(requested,cap);
            for (int i=0;i<12 && dv>0.05;i++,dv*=0.5)
                if (feasible(dv)) return dv;
            return 0;
        }

        internal static double UpdatePulseTrust(double prior, double errorBefore, double errorAfter,
            double expectedMovement, double driftAllowance)
        {
            if (!Finite(prior) || !Finite(errorBefore) || !Finite(errorAfter) ||
                !Finite(expectedMovement) || !Finite(driftAllowance) || expectedMovement<=0 ||
                driftAllowance<0) return 1;
            double progress=errorBefore-errorAfter-driftAllowance;
            return progress>=0.5*expectedMovement ? Math.Min(8,2*prior) : Math.Max(0.25,prior/2);
        }

        internal static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
    }

    // Owned by the approach, never by a replaceable predictor result or step.
    internal sealed class DunaBrakingCommitment
    {
        internal bool Committed { get; private set; }
        internal bool ForceFirst { get; private set; }
        internal double CommitUT { get; private set; }=double.NaN;
        internal string Reason { get; private set; }="none";
        internal void Commit(double now, string reason, bool forceFirst)
        {
            if (Committed) return;
            Committed=true; CommitUT=now; Reason=reason; ForceFirst=forceFirst;
        }
        internal bool RequiresThrust(bool poweredObserved, bool safeDelay, double speed) =>
            Committed && DunaBrakingHandoff.Finite(speed) && speed>DunaBrakingHandoff.CaptureSpeed &&
            (ForceFirst && !poweredObserved || !safeDelay);
    }
}
