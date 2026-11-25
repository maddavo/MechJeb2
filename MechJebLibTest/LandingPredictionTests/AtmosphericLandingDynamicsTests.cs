using System;
using MuMech.Landing;
using Xunit;
using D=MuMech.Landing.AtmosphericLandingDynamics;

namespace MechJebLibTest.LandingPredictionTests
{
    public class AtmosphericLandingDynamicsTests
    {
        private const double G0=9.80665;
        private static D.V X=>new D.V(1,0,0);
        private static D.Curve Isp()=>new D.Curve(new D.Knot(0,400),new D.Knot(1,280),new D.Knot(8,.001));
        private static D.Propulsion Engines(double flow=1,double limit=1)=>
            new D.Propulsion(true,new D.Engine(flow,limit,G0,default,Isp()));
        private static D.Environment Vacuum(double mu=0,double terrain=320000)=>
            new D.Environment(320000,mu,default,0,null,null,0,terrain,0);
        private static D.Limits Limits(double mass=100,double turn=10,double rise=0,double fall=0,double dt=.1)=>
            new D.Limits(mass,turn,rise,fall,dt);
        private static D.State State(double height,double speed,double mass=1000,double throttle=0)=>
            new D.State(0,X*(320000+height),X*speed,mass,X,throttle);

        [Fact]
        public void PressureChangesThrustButNotConstantMassFlow()
        {
            D.Authority a=Engines().At(0),b=Engines().At(101325);
            Assert.True(a.Known&&b.Known);
            Assert.Equal(400*G0,a.Force,8);
            Assert.Equal(.7*a.Force,b.Force,8);
            Assert.Equal(a.Flow,b.Flow,12);
            Assert.False(Engines().At(9*101325).Known);
        }
        [Fact]
        public void OpposedMomentArmsLimitSustainableForceAndFlow()
        {
            var p=new D.Propulsion(true,new D.Engine(2,1,G0,new D.V(2,0,0),Isp()),
                new D.Engine(2,1,G0,new D.V(-1,0,0),Isp()));
            D.Authority a=p.At(0);
            Assert.True(a.Known);
            Assert.Equal(3,a.Flow,10);
            Assert.Equal(3*400*G0,a.Force,8);
            Assert.InRange(a.TorqueResidual,0,1e-8);
            Assert.False(new D.Propulsion(false,new D.Engine(1,1,G0,default,Isp())).At(0).Known);
            Assert.False(new D.Propulsion(true,new D.Engine(1,1,G0,new D.V(1,0,0),Isp()),
                new D.Engine(1,1,G0,new D.V(0,0,1),Isp())).At(0).Known);
        }
        [Fact]
        public void PersistentLimitsAreNotInstantaneousTrim()
        {
            var p=new D.Propulsion(true,new D.Engine(2,.5,G0,new D.V(1,0,0),Isp()),
                new D.Engine(2,1,G0,new D.V(-1,0,0),Isp()));
            Assert.Equal(2,p.At(0).Flow,10);
        }
        [Fact]
        public void HermiteTangentsAndCurveDomainAreExplicit()
        {
            var c=new D.Curve(new D.Knot(0,0,0,0),new D.Knot(1,1,0,0));
            Assert.Equal(.15625,c.At(.25),10);
            Assert.True(double.IsNaN(c.At(1.1)));
        }
        private static D.State Burn(double dt)
        {
            var s=State(10000,0);
            for(int i=0;i<(int)Math.Round(10/dt);i++)
            {
                var a=D.Advance(Vacuum(),Engines(),Limits(dt:dt),s,new D.Command(1,X),dt);
                Assert.Equal(D.Outcome.Advanced,a.Status);
                s=a.End;
            }
            return s;
        }
        [Fact]
        public void FiniteBurnMatchesRocketEquationAndStepConverges()
        {
            D.State coarse=Burn(.5),fine=Burn(.1);
            double expected=400*G0*Math.Log(1000.0/990);
            Assert.Equal(990,fine.Mass,7);
            Assert.InRange(Math.Abs(fine.Velocity.X-expected),0,1e-4);
            Assert.True(Math.Abs(fine.Velocity.X-expected)<Math.Abs(coarse.Velocity.X-expected));
        }
        [Fact]
        public void PressureAndMassBothChangeAcceleration()
        {
            var p=new D.Curve(new D.Knot(0,101325),new D.Knot(1000,0));
            var rho=new D.Curve(new D.Knot(0,0),new D.Knot(1000,0));
            var env=new D.Environment(320000,0,default,1000,p,rho,0,320000,0);
            var a=D.Advance(env,Engines(),Limits(),State(900,-100),new D.Command(1,X),.1);
            var b=D.Advance(env,Engines(),Limits(),State(100,-100),new D.Command(1,X),.1);
            Assert.Equal(D.Outcome.Advanced,a.Status);
            Assert.Equal(D.Outcome.Advanced,b.Status);
            Assert.True(a.End.Velocity.X>b.End.Velocity.X);
            Assert.Equal(a.End.Mass,b.End.Mass,10);
        }
        [Fact]
        public void EngineTailAndAttitudeSlewAreFinite()
        {
            var s=State(10000,0,throttle:1);
            var a=D.Advance(Vacuum(),Engines(),Limits(turn:.5,fall:2),s,new D.Command(0,new D.V(0,1,0)),.1);
            Assert.Equal(D.Outcome.Advanced,a.Status);
            Assert.Equal(Math.Exp(-.05),a.End.Throttle,10);
            Assert.InRange(a.End.Mass,999.89,999.91);
            Assert.Equal(.05,Math.Acos(D.V.Dot(s.Forward,a.End.Forward)),8);
            Assert.InRange(a.End.PointingRate,.499999,.500001);
        }
        [Fact]
        public void RotatingAirAtRestHasNoDrag()
        {
            var p=new D.Curve(new D.Knot(0,0),new D.Knot(10000,0));
            var rho=new D.Curve(new D.Knot(0,1),new D.Knot(10000,1));
            var env=new D.Environment(320000,0,new D.V(0,0,.0001),10000,p,rho,10,320000,0);
            var r=X*321000;
            var v=D.V.Cross(env.Omega,r);
            var s=new D.State(0,r,v,1000,X);
            Assert.Equal(0,env.Surface(r,v).Length,10);
            var a=D.Advance(env,Engines(),Limits(dt:.01),s,new D.Command(0,X),.01);
            Assert.Equal(D.Outcome.Advanced,a.Status);
            Assert.InRange((a.End.Velocity-v).Length,0,1e-8);
        }
        [Fact]
        public void ContactAndReserveAreEventsNotSuccessByAltitude()
        {
            var a=D.Advance(Vacuum(),Engines(),Limits(dt:1),State(1,-10),new D.Command(0,X),1);
            Assert.Equal(D.Outcome.ContactUnsafe,a.Status);
            Assert.InRange(a.End.UT,.099999,.100001);
            var b=D.Advance(Vacuum(),Engines(),Limits(999.95,dt:1),State(10000,0),new D.Command(1,X),1);
            Assert.Equal(D.Outcome.ReserveExhausted,b.Status);
            Assert.InRange(b.End.UT,.049999,.050001);
        }
        [Fact]
        public void SubHoverInitialAuthorityDoesNotRejectBeforePropagation()
        {
            // Initial acceleration 2 m/s² < terrain g 2.94, but high above
            // terrain, finite fuel burn and lower local g allow speed capture.
            // This is synthetic and is NOT a validated touchdown.
            double mu=2.94*320000*320000.0;
            double flow=2000/(400*G0);
            var s=State(400000,-20);
            var a=D.Screen(Vacuum(mu),Engines(flow),Limits(dt:.25),s,0,new D.Command(0,X),0,60);
            Assert.Equal(D.Outcome.CaptureEnvelope,a.Status);
            Assert.True(a.End.UT>0);
            Assert.True(a.End.Mass<s.Mass);
        }
        [Fact]
        public void ActualFailedAndDelayedCandidatesHaveDistinctMeaning()
        {
            var s=State(1000,-100);
            var lim=Limits(dt:.1);
            var engines=Engines(10);
            var immediate=D.Screen(Vacuum(),engines,lim,s,0,new D.Command(0,X),0,30);
            var delayed=D.Screen(Vacuum(),engines,lim,s,0,new D.Command(0,X),15,30);
            Assert.Equal(D.Outcome.CaptureEnvelope,immediate.Status);
            Assert.Equal(D.Outcome.ContactUnsafe,delayed.Status);
            Assert.Equal(D.Window.EarlierCandidateOnly,D.Classify(0,2,20,immediate,delayed));
            Assert.Equal(D.Window.NoDemonstratedCandidate,D.Classify(0,2,20,delayed,delayed));
            Assert.Equal(D.Window.EntryReadinessExpired,D.Classify(0,20,20,immediate,immediate));
            Assert.Equal(D.Window.CandidateFits,D.Classify(0,2,20,immediate,immediate));
        }
        [Fact]
        public void MissingCapabilityIsNotPhysicalInfeasibility()
        {
            var s=State(1000,-100);
            var unknown=D.Screen(Vacuum(),new D.Propulsion(false),Limits(),s,0,new D.Command(0,X),0,30);
            Assert.Equal(D.Outcome.UnknownCapability,unknown.Status);
            Assert.Equal(D.Window.Unresolved,D.Classify(0,2,20,unknown,unknown));
        }
        [Fact]
        public void CorrectionIsFiniteAndEntryDeadlineIsIndependent()
        {
            var s=State(10000,-50);
            var a=D.Screen(Vacuum(),Engines(5),Limits(dt:.1),s,2,new D.Command(.5,X),4,60);
            Assert.Equal(D.Outcome.CaptureEnvelope,a.Status);
            Assert.True(a.End.Mass<s.Mass);
            Assert.Equal(0,a.FirstPoweredUT,9);
            Assert.Equal(D.Window.EntryReadinessExpired,D.Classify(0,4,3,a,a));
        }
    }
}
