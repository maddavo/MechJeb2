using System;
using MuMech.Landing;
using Xunit;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using R=MuMech.Landing.AtmosphericLandingReplay;
using T=MuMech.Landing.AtmosphericTerminalReplay;

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
        public void ExponentialEngineImpulseAndMassAreInvariantToStepPartition()
        {
            var env=Vacuum();var engine=Engines(350);
            var start=State(10000,0,220000);
            double tau=.1;
            D.State Run(double dt)
            {
                var lim=Limits(turn:10,rise:tau,fall:tau,dt:dt);
                var s=start;
                foreach(double throttle in new[]{1.0,0.0,0.65})
                    for(int i=0;i<(int)Math.Round(.5/dt);i++)
                    {
                        var r=D.Advance(env,engine,lim,s,new D.Command(throttle,X),dt);
                        Assert.Equal(D.Outcome.Advanced,r.Status);s=r.End;
                    }
                return s;
            }
            var coarse=Run(.5);var fine=Run(.03125);
            double attained=0,impulseTime=0;
            foreach(double throttle in new[]{1.0,0.0,.65})
            {
                impulseTime+=throttle*.5+(attained-throttle)*tau*(1-Math.Exp(-.5/tau));
                attained=throttle+(attained-throttle)*Math.Exp(-.5/tau);
            }
            double expectedMass=start.Mass-350*impulseTime;
            double expectedVelocity=400*G0*Math.Log(start.Mass/expectedMass);
            Assert.Equal(expectedMass,coarse.Mass,7);
            Assert.Equal(expectedMass,fine.Mass,7);
            Assert.Equal(attained,coarse.Throttle,12);
            Assert.InRange(Math.Abs(coarse.Velocity.X-expectedVelocity),0,1e-4);
            Assert.InRange((coarse.Velocity-fine.Velocity).Length,0,1e-4);
            Assert.InRange((coarse.R-fine.R).Length,0,1e-3);
        }

        [Fact]
        public void AtmosphericContactPrecedesOutOfDomainMidpoint()
        {
            var p=new D.Curve(new D.Knot(0,20000),new D.Knot(1000,0));
            var rho=new D.Curve(new D.Knot(0,.15),new D.Knot(1000,0));
            var env=new D.Environment(320000,0,default,1000,p,rho,0,320000,0);
            Assert.False(env.AirAtAltitude(-.001,out _,out _));
            var lim=Limits(dt:.2);
            var a=D.Advance(env,Engines(),lim,State(.13,-1.6),new D.Command(0,X),.2);
            Assert.Equal(D.Outcome.ContactUnsafe,a.Status);
            Assert.InRange(a.End.UT,.081249,.081251);
            Assert.InRange(env.PhysicalClearance(a.End),-1e-5,0);
            Assert.Equal(-1.6,a.End.Velocity.X,9);
            var b=D.Advance(env,Engines(),lim,State(.02,-.5),new D.Command(0,X),.2);
            Assert.Equal(D.Outcome.ContactWithinLimits,b.Status);
            Assert.InRange(b.End.UT,.039999,.040001);
        }

        [Fact]
        public void ContactLocalisationSurvivesPositionAndAbsoluteEpochRounding()
        {
            var p=new D.Curve(new D.Knot(0,20000),new D.Knot(1000,0));
            var rho=new D.Curve(new D.Knot(0,.15),new D.Knot(1000,0));
            var env=new D.Environment(320000,0,default,1000,p,rho,0,320000,0);
            const double ut=31154879;
            // Two radial double-precision ULPs above the surface. The
            // physical time to contact is below one absolute-epoch ULP.
            var start=new D.State(ut,X*(320000+1.1641532182693481e-10),X*(-.5),1000,X);
            Assert.True(env.PhysicalClearance(start)>0);
            var result=D.Advance(env,Engines(),Limits(dt:.2),start,new D.Command(0,X),.2);
            Assert.Equal(D.Outcome.ContactWithinLimits,result.Status);
            Assert.InRange(result.End.UT,ut,ut+1e-7);
            Assert.InRange(env.PhysicalClearance(result.End),-1e-5,0);
            Assert.Equal(-.5,result.End.Velocity.X,9);
        }

        [Fact]
        public void UnresolvedAtmosphereAboveSurfaceIsNotContact()
        {
            var env=new D.Environment(320000,0,default,1000,null,null,0,320000,0,
                atmosphere:(ut,r)=>r.Length>=320000.1?new D.AirSample(20000,.15):default);
            var start=State(.13,-1.6);
            var result=D.Advance(env,Engines(),Limits(dt:.2),start,new D.Command(0,X),.2);
            Assert.Equal(D.Outcome.UnknownCapability,result.Status);
            Assert.True(env.PhysicalClearance(result.End)>0);
        }

        [Fact]
        public void EndpointPointingRateDoesNotDependOnAverageOverStep()
        {
            var env=Vacuum();var engine=Engines();
            var target=new D.V(Math.Cos(.02),Math.Sin(.02),0);
            var lim=Limits(turn:1,dt:.1);
            var result=D.Advance(env,engine,lim,State(100,0),new D.Command(0,target),.1);
            Assert.Equal(D.Outcome.Advanced,result.Status);
            Assert.Equal(0,result.End.PointingRate);
            Assert.InRange((result.End.Forward-target).Length,0,1e-8);
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
            Assert.Equal(D.Window.EarlierCandidateOnly,D.Classify(0,2,20,15,3,immediate,delayed));
            Assert.Equal(D.Window.NoDemonstratedCandidate,D.Classify(0,2,20,15,3,delayed,delayed));
            Assert.Equal(D.Window.EntryReadinessExpired,D.Classify(0,20,20,25,3,immediate,immediate));
            Assert.Equal(D.Window.CandidateFits,D.Classify(0,2,20,15,3,immediate,immediate));
        }
        [Fact]
        public void MissingCapabilityIsNotPhysicalInfeasibility()
        {
            var s=State(1000,-100);
            var unknown=D.Screen(Vacuum(),new D.Propulsion(false),Limits(),s,0,new D.Command(0,X),0,30);
            Assert.Equal(D.Outcome.UnknownCapability,unknown.Status);
            Assert.Equal(D.Window.Unresolved,D.Classify(0,2,20,15,3,unknown,unknown));
        }
        [Fact]
        public void CorrectionIsFiniteAndEntryDeadlineIsIndependent()
        {
            var s=State(10000,-50);
            var a=D.Screen(Vacuum(),Engines(5),Limits(dt:.1),s,2,new D.Command(.5,X),4,60);
            Assert.Equal(D.Outcome.CaptureEnvelope,a.Status);
            Assert.True(a.End.Mass<s.Mass);
            Assert.Equal(0,a.FirstPoweredUT,9);
            Assert.Equal(D.Window.EntryReadinessExpired,D.Classify(0,4,3,10,3,a,a));
            Assert.Equal(D.Window.TestedBrakingPreparationDoesNotFit,D.Classify(0,4,30,6,3,a,a));
            Assert.Equal(D.Window.CandidateFits,D.Classify(0,4,30,8,3,a,a));
        }
        [Fact]
        public void CaptureCannotHideUnknownOrZeroSustainableAuthority()
        {
            var s=State(1000,-5);
            var unknown=D.Screen(Vacuum(),new D.Propulsion(false),Limits(),s,0,new D.Command(0,X),0,10);
            var zero=D.Screen(Vacuum(),new D.Propulsion(true,new D.Engine(1,1,G0,new D.V(1,0,0),Isp())),
                Limits(),s,0,new D.Command(0,X),0,10);
            Assert.Equal(D.Outcome.UnknownCapability,unknown.Status);
            Assert.Equal(D.Outcome.NoControllableThrust,zero.Status);
        }
        [Fact]
        public void SafeDirectContactDoesNotRequireHoverAuthority()
        {
            var s=State(.05,-.1);
            var a=D.Screen(Vacuum(2.94*320000*320000.0),Engines(.01),Limits(dt:.1),s,0,new D.Command(0,X),1,10);
            Assert.Equal(D.Outcome.ContactWithinLimits,a.Status);
            Assert.InRange(a.End.Velocity.X,-1,0);
            Assert.True(a.End.UT<1);
        }
        [Fact]
        public void SuppliedPassiveLiftIsForceNotFreeSteering()
        {
            var p=new D.Curve(new D.Knot(0,0),new D.Knot(1000,0));
            var rho=new D.Curve(new D.Knot(0,1),new D.Knot(1000,1));
            var env=new D.Environment(320000,0,default,1000,p,rho,0,320000,0,
                (r,w,f,pressure,density)=>new D.V(1000,0,0));
            var s=new D.State(0,X*320500,new D.V(0,20,0),1000,X);
            var a=D.Advance(env,Engines(),Limits(),s,new D.Command(0,X),.1);
            Assert.Equal(D.Outcome.Advanced,a.Status);
            Assert.Equal(.1,a.End.Velocity.X,10);
            Assert.Equal(20,a.End.Velocity.Y,10);
        }

        [Fact]
        public void ForceDecompositionHasCorrectUnitsAndRotation()
        {
            var pressure=new D.Curve(new D.Knot(0,101325),new D.Knot(10000,101325));
            var density=new D.Curve(new D.Knot(0,1),new D.Knot(10000,1));
            var r=X*321000;
            double mu=2.5*r.Length*r.Length;
            var env=new D.Environment(320000,mu,new D.V(0,0,.0001),10000,pressure,density,2,320000,0,
                (pos,air,f,p,rho)=>new D.V(0,0,100));
            var v=D.V.Cross(env.Omega,r)+new D.V(0,20,0);
            var s=new D.State(0,r,v,1000,X,.5);
            D.ForceSample forces=D.Forces(env,Engines(2),s);
            Assert.True(forces.Known);
            Assert.Equal(101325,forces.Pressure,8);
            Assert.Equal(1,forces.Density,8);
            Assert.Equal(20,forces.AirVelocity.Y,8);
            Assert.Equal(-2.5,forces.Gravity.X,8);
            Assert.Equal(280*G0,forces.Thrust.X,8);
            Assert.Equal(-400,forces.Drag.Y,8);
            Assert.Equal(100,forces.Lift.Z,8);
            Assert.Equal(1,forces.MassFlow,8);
            Assert.Equal(-2.5+280*G0/1000,forces.Acceleration.X,8);
            Assert.Equal(-.4,forces.Acceleration.Y,8);
            Assert.Equal(.1,forces.Acceleration.Z,8);
        }

        [Fact]
        public void ExplicitAtmosphereAndAeroAdaptersUseEpochPositionAndAttainedPointing()
        {
            var samples=new System.Collections.Generic.List<(double ut,D.V forward)>();
            var env=new D.Environment(320000,0,default,10000,null,null,0,320000,0,
                atmosphere:(ut,r)=>new D.AirSample(ut*1000+r.Length-320000,1),
                aerodynamics:(ut,r,w,f,p,rho)=>{
                    samples.Add((ut,f));
                    return new D.AeroSample(w*(-1),new D.V(0,0,f.Y*100));
                });
            var s=new D.State(10,X*321000,new D.V(0,20,0),1000,X);
            var step=D.Advance(env,Engines(),Limits(turn:1),s,new D.Command(1,new D.V(0,1,0)),.1);
            Assert.Equal(D.Outcome.Advanced,step.Status);
            Assert.NotEmpty(samples);
            Assert.Contains(samples,sample=>Math.Abs(sample.ut-10.05)<1e-9);
            Assert.All(samples,sample=>{
                Assert.InRange(sample.ut,10,10.1);
                Assert.Equal(Math.Sin(sample.ut-10),sample.forward.Y,8);
            });
            D.ForceSample f1=D.Forces(env,Engines(),step.End);
            Assert.True(f1.Known);
            Assert.Equal(step.End.UT*1000+step.End.R.Length-320000,f1.Pressure,8);
            Assert.Equal(100*step.End.Forward.Y,f1.Lift.Z,8);
        }

        [Fact]
        public void ExplicitUnknownAeroDoesNotFallBackToZero()
        {
            var env=new D.Environment(320000,0,default,10000,null,null,0,320000,0,
                atmosphere:(ut,r)=>new D.AirSample(0,1),
                aerodynamics:(ut,r,w,f,p,rho)=>default);
            Assert.False(D.Forces(env,Engines(),State(1000,-20)).Known);
            Assert.Equal(D.Outcome.UnknownCapability,
                D.Advance(env,Engines(),Limits(),State(1000,-20),new D.Command(0,X),.1).Status);
            var missingAir=new D.Environment(320000,0,default,10000,null,null,0,320000,0,
                atmosphere:(ut,r)=>default);
            Assert.False(D.Forces(missingAir,Engines(),State(1000,-20)).Known);
        }

        [Fact]
        public void InvalidAeroCannotAddEnergyThroughPassiveDragOrDoubleCount()
        {
            var env=new D.Environment(320000,0,default,10000,null,null,0,320000,0,
                atmosphere:(ut,r)=>new D.AirSample(0,1),
                aerodynamics:(ut,r,w,f,p,rho)=>new D.AeroSample(w,default));
            Assert.False(D.Forces(env,Engines(),State(1000,-20)).Known);
            var ambiguous=new D.Environment(320000,0,default,10000,null,null,1,320000,0,
                atmosphere:(ut,r)=>new D.AirSample(0,1),
                aerodynamics:(ut,r,w,f,p,rho)=>new D.AeroSample(default,default));
            Assert.False(ambiguous.Valid);
        }

        [Fact]
        public void ExactScalarAltitudeRetainsStrictPressureDomain()
        {
            var c=new D.Curve(new D.Knot(0,10),new D.Knot(1000,0));
            var env=new D.Environment(320000,0,default,1000,c,c,0,320000,0);
            Assert.True(env.AirAtAltitude(0,out double p,out _));
            Assert.Equal(10,p);
            Assert.False(env.AirAtAltitude(-1e-9,out _,out _));
        }

        [Fact]
        public void ReplayFiniteBurnAndCoastConvergeWithoutResettingToObservation()
        {
            var initial=State(10000,0);
            var commands=new[]{new R.Segment(0,10,new D.Command(1,X)),new R.Segment(10,20,new D.Command(0,X))};
            double speed=400*G0*Math.Log(1000.0/990);
            // Integral of the rocket-equation speed, plus ten-second coast.
            double displacement=400*G0*(10+990*Math.Log(.99))+10*speed;
            var observation=new R.Observation(new D.State(20,X*(330000+displacement),X*speed,990,X));
            var coarse=R.Run(Vacuum(),Engines(),Limits(dt:.5),initial,commands,new[]{observation});
            var fine=R.Run(Vacuum(),Engines(),Limits(dt:.25),initial,commands,new[]{observation});
            Assert.True(coarse.HistoryCompleted&&fine.HistoryCompleted);
            Assert.Equal(2,fine.CompletedSegments);
            Assert.Equal(990,fine.End.Mass,8);
            Assert.InRange((fine.End.R-coarse.End.R).Length,0,1);
            Assert.InRange((fine.End.Velocity-coarse.End.Velocity).Length,0,.1);
            Assert.InRange(fine.Comparisons[0].PositionError,0,1);
            Assert.InRange(fine.Comparisons[0].VelocityError,0,.1);
            Assert.True(fine.Comparisons[0].PositionError<coarse.Comparisons[0].PositionError);
            Assert.False(fine.Comparisons[0].ForceResidualKnown);
            Assert.True(double.IsNaN(fine.Comparisons[0].FlowResidual));
        }

        [Fact]
        public void ReplayUsesContinuousHistoryAndSeparatesForceModelFromStateResidual()
        {
            var initial=State(1000,10);
            var history=new[]{new R.Segment(0,2,new D.Command(0,X))};
            // Deliberately different observed state; propagation must not reset.
            var a=new R.Observation(new D.State(.75,X*321100,X*100,1000,X,1),
                X*(400*G0-100),default,default,.8);
            var b=new R.Observation(new D.State(2,X*321020,X*10,1000,X));
            var run=R.Run(Vacuum(),Engines(),Limits(dt:.5),initial,history,new[]{a,b});
            Assert.True(run.HistoryCompleted);
            Assert.Equal(2,run.Comparisons.Count);
            Assert.Equal(.75,run.Comparisons[0].UT,10);
            Assert.Equal(92.5,run.Comparisons[0].PositionError,8);
            Assert.Equal(90,run.Comparisons[0].VelocityError,8);
            Assert.True(run.Comparisons[0].ForceResidualKnown);
            Assert.Equal(100,run.Comparisons[0].ThrustResidual.X,8);
            Assert.Equal(.2,run.Comparisons[0].FlowResidual,8);
            Assert.Equal(0,run.Comparisons[0].PredictedForces.Thrust.Length,8);
            Assert.Equal(0,run.Comparisons[1].PositionError,8);
        }

        [Fact]
        public void ReplayRejectsUnspecifiedIntervalsAndOutOfOrderObservations()
        {
            var s=State(1000,10);
            var off=new D.Command(0,X);
            var gap=new[]{new R.Segment(0,1,off),new R.Segment(1.1,2,off)};
            Assert.Equal(D.Outcome.InvalidInput,
                R.Run(Vacuum(),Engines(),Limits(),s,gap,Array.Empty<R.Observation>()).Outcome);
            var history=new[]{new R.Segment(0,2,off)};
            var a=new R.Observation(new D.State(1,X*321000,X*10,1000,X));
            Assert.Equal(D.Outcome.InvalidInput,R.Run(Vacuum(),Engines(),Limits(),s,history,new[]{a,a}).Outcome);
        }

        [Fact]
        public void ReplayDoesNotContinueThroughContactOrClaimCaptureIsLanding()
        {
            var s=State(1,-10);
            var history=new[]{new R.Segment(0,10,new D.Command(0,X))};
            var result=R.Run(Vacuum(),Engines(),Limits(dt:.5),s,history,Array.Empty<R.Observation>());
            Assert.Equal(D.Outcome.ContactUnsafe,result.Outcome);
            Assert.False(result.HistoryCompleted);
            Assert.InRange(result.End.UT,.099999,.100001);
            var high=R.Run(Vacuum(),Engines(),Limits(dt:.5),State(1000,0),history,Array.Empty<R.Observation>());
            Assert.True(high.HistoryCompleted);
            Assert.Equal(D.Outcome.Advanced,high.Outcome);
        }

        [Fact]
        public void ReplayStopsOnCancellationBudgetOrUnknownModel()
        {
            var s=State(1000,10);
            var h=new[]{new R.Segment(0,10,new D.Command(0,X))};
            var observations=Array.Empty<R.Observation>();
            var cancelled=R.Run(Vacuum(),Engines(),Limits(),s,h,observations,
                cancellation:new System.Threading.CancellationToken(true));
            Assert.Equal(D.Outcome.Cancelled,cancelled.Outcome);
            Assert.Equal(0,cancelled.Steps);
            var bounded=R.Run(Vacuum(),Engines(),Limits(),s,h,observations,maxSteps:1);
            Assert.Equal(D.Outcome.Horizon,bounded.Outcome);
            Assert.Equal(1,bounded.Steps);
            var unknown=R.Run(Vacuum(),new D.Propulsion(false),Limits(),s,h,observations);
            Assert.Equal(D.Outcome.UnknownCapability,unknown.Outcome);
            Assert.False(unknown.HistoryCompleted);
        }

        private static D.Environment Terrain(Func<D.State,D.SurfaceSample> sample,
            Func<D.State,D.State,D.Command,double> bound)
            =>new D.Environment(320000,0,default,0,null,null,0,320000,0,
                terrain:new D.TerrainModel(sample,bound));

        private static D.SurfaceSample PlaneSample(D.State s,double plane=320005,bool landable=true)
            =>new D.SurfaceSample(s.R.X-plane,s.R.X-plane-2,s.R.X-320000,X,landable);

        [Fact]
        public void TerrainContactIsNotControllerAltitudeOrDatum()
        {
            // Analytic flat support plane; zero gravity and thrust, normal descent.
            var env=Terrain(s=>PlaneSample(s),(a,b,c)=>Math.Min(a.R.X,b.R.X)-320005);
            var start=State(5.5,-.5);
            Assert.Equal(.5,env.SurfaceAt(start).Clearance,10);
            Assert.Equal(-1.5,env.SurfaceAt(start).ControllerClearance,10);
            var result=D.Advance(env,Engines(),Limits(dt:1),start,new D.Command(0,X),1);
            Assert.Equal(D.Outcome.ContactWithinLimits,result.Status);
            Assert.InRange(result.End.UT,.999999,1.000001);
            Assert.InRange(result.End.R.X,320004.999999,320005.000001);
        }

        [Fact]
        public void TerrainUnsafeSupportAndContactNormalAreExplicit()
        {
            var slope=new D.V(.8,.6,0);
            var env=Terrain(s=>new D.SurfaceSample(s.R.X-320005,0,5,slope,true),
                (a,b,c)=>Math.Min(a.R.X,b.R.X)-320005);
            var result=D.Advance(env,Engines(),Limits(dt:1),State(5.5,-.5),new D.Command(0,X),1);
            Assert.Equal(D.Outcome.ContactUnsafe,result.Status);
            var water=Terrain(s=>PlaneSample(s,landable:false),(a,b,c)=>Math.Min(a.R.X,b.R.X)-320005);
            Assert.Equal(D.Outcome.ContactUnsafe,D.Advance(water,Engines(),Limits(dt:1),
                State(5.5,-.5),new D.Command(0,X),1).Status);
        }

        [Fact]
        public void TerrainUnknownOrContradictoryBoundNeverPublishesLanding()
        {
            var env=Terrain(s=>PlaneSample(s),(a,b,c)=>double.NaN);
            var s=State(5.5,-.5);
            var result=D.Advance(env,Engines(),Limits(dt:1),s,new D.Command(0,X),1);
            Assert.Equal(D.Outcome.UnknownTerrain,result.Status);
            var bad=Terrain(t=>PlaneSample(t),(a,b,c)=>100);
            Assert.Equal(D.Outcome.UnknownTerrain,D.Advance(bad,Engines(),Limits(dt:1),s,new D.Command(0,X),1).Status);
            Assert.Equal(D.Window.Unresolved,D.Classify(0,0,100,1,0,
                new D.Result(D.Outcome.CaptureEnvelope,s,100),result));
        }

        [Fact]
        public void SweptInteriorRidgeIsDetectedBeforeClearEndpoint()
        {
            // x is radial; y crosses a triangular 2 m ridge. Analytic
            // linear path; interval max ridge height at clamped y=0.
            double Ridge(double y)=>2*Math.Max(0,1-Math.Abs(y));
            var env=Terrain(s=>new D.SurfaceSample(s.R.X-320000-Ridge(s.R.Y),10,10,X,false),
                (a,b,c)=>{
                    double lo=Math.Min(a.R.Y,b.R.Y),hi=Math.Max(a.R.Y,b.R.Y);
                    double y=Math.Max(lo,Math.Min(hi,0));
                    return Math.Min(a.R.X,b.R.X)-320000-Ridge(y);
                });
            var s=new D.State(0,new D.V(320001,-2,0),new D.V(0,4,0),1000,X);
            var step=D.Advance(env,Engines(),Limits(dt:1),s,new D.Command(0,X),1);
            Assert.Equal(D.Outcome.ContactUnsafe,step.Status);
            Assert.InRange(step.End.UT,.374999,.375001);
        }

        [Fact]
        public void TerrainReserveExhaustionPrecedesLaterContact()
        {
            // Radial plane, off-axis force so radial path stays linear.
            var env=Terrain(s=>PlaneSample(s),(a,b,c)=>Math.Min(a.R.X,b.R.X)-320005);
            var y=new D.V(0,1,0);
            var s=new D.State(0,X*320006,X*(-1),1000,y,1);
            var step=D.Advance(env,Engines(),Limits(999.75,dt:1),s,new D.Command(1,y),1);
            Assert.Equal(D.Outcome.ReserveExhausted,step.Status);
            Assert.InRange(step.End.UT,.249999,.250001);
        }

        private static T.Settings TerminalSettings(double tick=.02,double touchdown=1,double gee=.3,
            double raw=10,double limited=10,double cap=1,double angle=0,double smoothing=0,bool torque=false,
            Func<double,D.V,double> terrain=null)
            =>new T.Settings(tick,touchdown,gee,smoothing,
                s=>new T.ControllerAuthority(raw,limited,cap,torque),(s,d)=>angle,terrain);

        [Fact]
        public void TerminalKHVUsesAttainedDirectionAndPreservesSpeedPIDMemory()
        {
            var s=new D.State(0,X*320100,new D.V(-2,3,0),1000,X);
            var m=new T.Memory(T.Phase.HorizontalKill,T.Mode.Off,.1,.2,.7,.6);
            var tick=T.Tick(Vacuum(3*320100*320100.0),TerminalSettings(),s,m);
            Assert.True(tick.Known);
            Assert.Equal(.5,tick.RequestedThrottle,10);
            Assert.Equal(-.2/Math.Sqrt(1.04),tick.Command.Direction.Y,10);
            Assert.Equal(.7,tick.Next.PreviousThrottle,10);
            var quiet=T.Tick(Vacuum(),TerminalSettings(),State(100,-2),m);
            Assert.Equal(T.Phase.FinalDescent,quiet.Next.Phase);
            Assert.Equal(0,quiet.RequestedThrottle,10);
            Assert.Equal(.7,quiet.Next.PreviousThrottle,10);
        }

        [Fact]
        public void TerminalPIDUsesRawAuthorityLimitedEnvelopeAndPreviousRequest()
        {
            var env=Vacuum(3*320010*320010.0);
            var s=State(10,-4,throttle:.1);
            var m=new T.Memory(T.Phase.FinalDescent,T.Mode.KeepVertical,.1,.2,.7,.5);
            var settings=TerminalSettings(raw:10,limited:4,cap:.6,smoothing:1);
            var tick=T.Tick(env,settings,s,m);
            double set=-.9*Math.Sqrt(20);
            double error=(set+4)/10;
            double integral=.2+error*.02;
            double req=Math.Max(0,Math.Min(1,.7+.05*error+.000001*integral+.05*(error-.1)/.02));
            Assert.True(tick.Known);
            Assert.Equal(set,tick.Setpoint,9);
            Assert.Equal(req,tick.RequestedThrottle,9);
            Assert.Equal(req,tick.Next.PreviousThrottle,9);
            Assert.InRange(tick.Command.Throttle,.48,.52);
            Assert.Equal(integral,tick.Next.Integral,10);
        }

        [Fact]
        public void TerminalAttitudeGateAndModeResetAreSourceDerived()
        {
            var env=Vacuum(3*320010*320010.0);
            var s=State(10,-3);
            var m=new T.Memory(T.Phase.FinalDescent,T.Mode.Direct,100,200,.6,.6);
            var tick=T.Tick(env,TerminalSettings(angle:100,torque:true),s,m);
            Assert.True(tick.Known);
            Assert.Equal(.1,tick.RequestedThrottle,10);
            Assert.Equal(T.Mode.KeepVertical,tick.Next.Mode);
            Assert.InRange(Math.Abs(tick.Next.Integral),0,.1);
            var noTorque=T.Tick(env,TerminalSettings(angle:100),s,m);
            Assert.Equal(0,noTorque.RequestedThrottle,10);
        }

        [Fact]
        public void TerminalUpperBranchRequiresActualProjectedSurface()
        {
            var s=State(1000,-20);
            var m=new T.Memory(T.Phase.FinalDescent,T.Mode.KeepVertical,0,0,.5,.5);
            var env=Vacuum(3*321000*321000.0);
            Assert.False(T.Tick(env,TerminalSettings(),s,m).Known);
            var tick=T.Tick(env,TerminalSettings(terrain:(ut,pos)=>320000),s,m);
            Assert.True(tick.Known);
            Assert.Equal(T.Mode.KeepSurface,tick.Next.Mode);
            Assert.True(tick.Setpoint>0);
            var ascending=T.Tick(env,TerminalSettings(),State(1000,20),m);
            Assert.Equal(T.Mode.Direct,ascending.Next.Mode);
            Assert.Equal(0,ascending.RequestedThrottle,10);
        }

        [Fact]
        public void TerminalSubstepsDoNotRepeatPIDAndContactIsExplicit()
        {
            // Zero cap/zero gravity makes the physical motion analytic.
            var s=State(.5,-.5);
            var m=new T.Memory(T.Phase.FinalDescent,T.Mode.KeepVertical,0,0,0,0);
            var settings=TerminalSettings(tick:.2,cap:0);
            var coarse=T.Run(Vacuum(),Engines(),Limits(dt:.2),settings,s,m,10,1,1,.1,.1);
            var fine=T.Run(Vacuum(),Engines(),Limits(dt:.05),settings,s,m,10,1,1,.1,.1);
            Assert.Equal(D.Outcome.ContactWithinLimits,coarse.Outcome);
            Assert.Equal(D.Outcome.ContactWithinLimits,fine.Outcome);
            Assert.InRange(coarse.End.UT,.999999,1.000001);
            Assert.InRange(fine.End.UT,.999999,1.000001);
            Assert.InRange(Math.Abs(coarse.Memory.Integral-fine.Memory.Integral),0,.011);
        }

        [Fact]
        public void TerrainRoundedBoundCannotHideSampledContact()
        {
            // Lower bound off by tolerated floating point roundoff at
            // exact contact: still must stop rather than advance through.
            var env=Terrain(s=>PlaneSample(s),(a,b,c)=>Math.Min(a.R.X,b.R.X)-320005+1e-9);
            var result=D.Advance(env,Engines(),Limits(dt:1),State(5.5,-.5),new D.Command(0,X),1);
            Assert.Equal(D.Outcome.ContactWithinLimits,result.Status);
        }

        [Fact]
        public void TerminalIntegrationBudgetBoundsSubstepsIndependentlyOfControllerTicks()
        {
            var m=new T.Memory(T.Phase.FinalDescent,T.Mode.KeepVertical,0,0,0,0);
            var run=T.Run(Vacuum(),Engines(),Limits(dt:.001),TerminalSettings(tick:.2,cap:0),
                State(100,-.5),m,10,1,1,.1,.1,maxIntegrationSteps:3);
            Assert.Equal(D.Outcome.Horizon,run.Outcome);
            Assert.Equal(1,run.Ticks);
            Assert.Equal(3,run.IntegrationSteps);
            Assert.Equal(.003,run.End.UT,12);
        }
    }
}
