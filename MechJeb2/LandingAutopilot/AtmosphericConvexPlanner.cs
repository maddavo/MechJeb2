using System;
using System.Threading;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;

namespace MuMech.Landing
{
    // C: fixed-time local convex minimum-energy acceleration subproblem.
    // Dykstra projection of zero onto an affine endpoint constraint and a
    // product of norm balls/pointing cones. Frozen passive acceleration
    // and mass/thrust: NOT a complete SOCP PDG or full SCvx implementation.
    // Re-linearise at each nonlinear feedback sample; reject unresolved
    // equality residuals rather than invent virtual control.
    internal static class AtmosphericConvexPlanner
    {
        internal readonly struct Solution
        {
            internal readonly bool Converged;
            internal readonly D.V[] Controls;
            internal readonly double PositionResidual,VelocityResidual,MaxConstraintViolation;
            internal readonly int Iterations;
            internal Solution(bool converged,D.V[] controls,double position,double velocity,double violation,int iterations)
            {Converged=converged;Controls=controls;PositionResidual=position;VelocityResidual=velocity;
                MaxConstraintViolation=violation;Iterations=iterations;}
        }
        // Euclidean projection onto a circular cone intersected with a
        // centred unit ball. Cone half-angle <= pi/2; unrestricted ball
        // is represented explicitly by angle pi.
        internal static D.V ProjectControl(D.V x,D.V axis,double angle)
        {
            if(!x.Valid||!axis.Valid||Math.Abs(axis.Length-1)>1e-6||!D.Finite(angle)||
                angle<0||(angle>Math.PI/2&&angle!=Math.PI))return new D.V(double.NaN,0,0);
            D.V y=x;
            if(angle<Math.PI)
            {
                double axial=D.V.Dot(x,axis);
                D.V side=x-axis*axial;
                double r=side.Length,c=Math.Cos(angle),s=Math.Sin(angle);
                if(axial<0||r*c>axial*s)
                {
                    double d=axial*c+r*s;
                    y=d<=0?default:axis*(d*c)+(r>0?side*(d*s/r):default);
                }
            }
            return y.Length>1?y.Unit:y;
        }

        // Controls are normalised by aMax and time by T. Exact piecewise
        // constant double-integrator endpoint rows: b_i=1/N and
        // a_i=(N-i-0.5)/N². Equality projection uses a 2x2 Gram inverse.
        internal static Solution Solve(D.V displacement,D.V velocityChange,D.V passive,
            double seconds,double aMax,D.V axis,double halfAngle,int nodes=12,int iterations=160,
            double tolerance=1e-5,CancellationToken token=default)
        {
            Solution Fail(int n=0)=>new Solution(false,Array.Empty<D.V>(),double.NaN,double.NaN,double.NaN,n);
            if(!displacement.Valid||!velocityChange.Valid||!passive.Valid||!D.Finite(seconds)||seconds<=0||
                !D.Finite(aMax)||aMax<=0||!axis.Valid||Math.Abs(axis.Length-1)>1e-6||
                !D.Finite(halfAngle)||halfAngle<0||(halfAngle>Math.PI/2&&halfAngle!=Math.PI)||
                nodes<2||nodes>64||iterations<1||iterations>1000||!D.Finite(tolerance)||tolerance<=0)
                return Fail();
            double dt=1.0/nodes;
            D.V rp=(displacement-passive*(.5*seconds*seconds))*(1/(aMax*seconds*seconds));
            D.V rv=(velocityChange-passive*seconds)*(1/(aMax*seconds));
            if(!rp.Valid||!rv.Valid||rp.Length>.5+tolerance||rv.Length>1+tolerance)return Fail();
            var weights=new double[nodes];double aa=0,ab=0,bb=nodes*dt*dt;
            for(int i=0;i<nodes;i++){double w=(nodes-i-.5)*dt*dt;weights[i]=w;aa+=w*w;ab+=w*dt;}
            double det=aa*bb-ab*ab;
            if(!(det>0))return Fail();
            var x=new D.V[nodes];var affineCorrection=new D.V[nodes];var coneCorrection=new D.V[nodes];
            var z=new D.V[nodes];double pe=double.PositiveInfinity,ve=pe,violation=pe;
            int n;
            for(n=0;n<iterations;n++)
            {
                if(token.IsCancellationRequested)return Fail(n);
                D.V p=default,v=default;
                for(int i=0;i<nodes;i++){z[i]=x[i]+affineCorrection[i];p+=z[i]*weights[i];v+=z[i]*dt;}
                D.V dp=rp-p,dv=rv-v;
                D.V lambda=(dp*bb-dv*ab)*(1/det),mu=(dv*aa-dp*ab)*(1/det);
                p=default;v=default;violation=0;double change=0;
                for(int i=0;i<nodes;i++)
                {
                    D.V affine=z[i]+lambda*weights[i]+mu*dt;
                    affineCorrection[i]=z[i]-affine;
                    D.V before=affine+coneCorrection[i];
                    D.V next=ProjectControl(before,axis,halfAngle);
                    if(!next.Valid)return Fail(n+1);
                    coneCorrection[i]=before-next;change=Math.Max(change,(next-x[i]).Length);x[i]=next;
                    p+=next*weights[i];v+=next*dt;
                    violation=Math.Max(violation,Math.Max(0,next.Length-1));
                    if(halfAngle<Math.PI)violation=Math.Max(violation,
                        Math.Max(0,next.Length*Math.Cos(halfAngle)-D.V.Dot(next,axis)));
                }
                pe=(p-rp).Length;ve=(v-rv).Length;
                if(!D.Finite(pe)||!D.Finite(ve)||!D.Finite(violation))return Fail(n+1);
                if(pe<=tolerance&&ve<=tolerance&&violation<=tolerance&&change<=tolerance)
                    return new Solution(true,x,pe*aMax*seconds*seconds,ve*aMax*seconds,violation,n+1);
            }
            // Even if a bounded iterate happens to satisfy the endpoint,
            // no converged solution is claimed if the iteration gate fails.
            return new Solution(false,x,pe*aMax*seconds*seconds,ve*aMax*seconds,violation,n);
        }

        internal static D.Command Command(D.Environment env,D.Propulsion propulsion,D.State state,
            F.Settings settings,D.V target,D.V terminalVelocity,double remaining,
            out double requested,out double saturation,CancellationToken token)
        {
            requested=saturation=double.NaN;
            var force=D.Forces(env,propulsion,state);
            if(!force.Known)return default;
            var authority=propulsion.At(force.Pressure);
            if(!authority.Known||authority.Force<=0)return default;
            double raw=authority.Force/state.Mass,cap=settings.Common.CommandCap;
            if(cap==0){requested=saturation=0;return new D.Command(0,state.Forward);}
            D.V passive=force.Gravity+(force.Drag+force.Lift)*(1/state.Mass);
            double clearance=env.PhysicalClearance(state);
            if(!D.Finite(clearance))return default;
            // Fixed local cone near terrain, unconstrained ball at altitude.
            // Actual slew and engine response are in the nonlinear model.
            double angle=clearance<=300?Math.PI/3:Math.PI;
            // In addition to scaled convergence, cap physical endpoint
            // residuals (5 m and 0.05 m/s) on long horizons. A local
            // frozen-force solution is still only a nonlinear search seed.
            double scale=raw*cap;
            double tolerance=Math.Min(1e-5,Math.Min(5/(scale*remaining*remaining),.05/(scale*remaining)));
            var solved=Solve(target-state.R-state.Velocity*remaining,terminalVelocity-state.Velocity,
                passive,remaining,scale,state.R.Unit,angle,12,160,tolerance,token);
            if(!solved.Converged||solved.PositionResidual>5||solved.VelocityResidual>.05)return default;
            D.V acceleration=solved.Controls[0]*(raw*cap);
            requested=acceleration.Length;saturation=Math.Max(0,requested/raw-cap);
            D.V direction=requested>1e-9?acceleration.Unit:state.R.Unit;
            double throttle=D.V.Dot(state.Forward,direction)>=.75?Math.Min(cap,requested/raw):0;
            return new D.Command(throttle,direction);
        }
    }
}
