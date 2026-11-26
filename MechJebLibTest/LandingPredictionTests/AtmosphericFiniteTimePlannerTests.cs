using System;
using System.Linq;
using System.Threading;
using Xunit;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using T=MuMech.Landing.AtmosphericTerminalReplay;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;
using Tracker=MuMech.Landing.AtmosphericPlanTracker;
using Replanning=MuMech.Landing.AtmosphericReplanning;
using R=MuMech.Landing.AtmosphericLandingReplay;

namespace MechJebLibTest.LandingPredictionTests
{
    public class AtmosphericFiniteTimePlannerTests
    {
        private static D.V X=>new D.V(1,0,0);
        private static D.V Y=>new D.V(0,1,0);
        private static D.Environment Env(double mu=0)=>new D.Environment(320000,mu,default,0,null,null,0,320000,0);
        private static D.Propulsion Engine(double flow=1)=>new D.Propulsion(true,
            new D.Engine(flow,1,9.80665,default,new D.Curve(new D.Knot(0,400))));
        private static D.Limits Limits(double reserve=100,double step=.1)=>new D.Limits(reserve,20,0,0,step);
        private static D.State Initial(double altitude=.53,double speed=-.5)=>new D.State(0,X*(320000+altitude),X*speed,1000,X);
        private static F.Settings Settings(double cap=0,double horizon=4,double coast=.4,double min=.5,double max=1,
            int candidates=128,int steps=10000,double tolerance=1,D.V? target=null)
        {
            var t=new T.Settings(.2,.5,0,0,s=>new T.ControllerAuthority(4,4*cap,cap,false),(s,d)=>0,(u,p)=>320000);
            var m=new T.Memory(T.Phase.FinalDescent,T.Mode.Off,0,0,0,0);
            var p=new P.Settings(target??X,horizon,coast,tolerance,t,m,intervals:2,refine:0,
                maxCandidates:candidates,maxSteps:steps,captureClearance:0,safeAngle:.1,safeRate:.1,commandCap:cap);
            return new F.Settings(p,320000,approachHeight:15,approachSpeed:.5,minPowered:min,maxPowered:max,
                controlTick:.2,coastIntervals:2,durationIntervals:2,refinements:1);
        }
        [Fact]
        public void EndpointLawMatchesAnalyticCubicAndUsesBothConstraints()
        {
            // x(t) = 3t²-2t³, x(0)=v(0)=v(1)=0, x(1)=1
            var a=F.EndpointAcceleration(default,default,X,default,1);
            Assert.Equal(6,a.X,12);
            var b=F.EndpointAcceleration(default,X,X,X,1);
            Assert.InRange(b.Length,0,1e-12);
            var c=F.EndpointAcceleration(default,default,X,Y,1);
            Assert.Equal(-2,c.Y,12);
            Assert.False(F.EndpointAcceleration(default,default,X,default,0).Valid);
        }
        [Fact]
        public void DirectContactHasFullOrderedCommandHistoryAndFineReplay()
        {
            var env=Env();var p=Engine();var lim=Limits();var s=Initial();
            var set=Settings();
            var a=F.Evaluate(env,p,lim,s,set,new F.Profile(0,.5),10000,true);
            Assert.True(a.Result.Feasible);
            Assert.Equal(0,a.Result.TerminalStartUT);
            Assert.NotEmpty(a.Reference);
            Assert.Equal(a.Reference.Count,a.Commands.Count);
            for(int i=0;i<a.Commands.Count;i++)
            {
                Assert.Equal(a.Reference[i].State.UT,a.Commands[i].StartUT);
                Assert.Equal(a.Reference[i].ValidUntil,a.Commands[i].EndUT);
                if(i>0)Assert.Equal(a.Commands[i-1].EndUT,a.Commands[i].StartUT);
            }
            Assert.True(F.Validate(env,p,lim,s,set,a,10000,.01,.01,out var reference,out var replay));
            Assert.Equal(D.Outcome.ContactWithinLimits,replay.Outcome);
            Assert.InRange(reference.Result.End.UT,1.05999,1.06001);
        }
        [Fact]
        public void BudgetAndCancellationNeverPromoteIncompleteTrajectory()
        {
            var b=F.Search(Env(),Engine(),Limits(),Initial(),Settings(steps:1));
            Assert.Equal(P.SearchStatus.BudgetExhausted,b.Status);
            Assert.Null(b.Best);
            Assert.Equal(1,b.Steps);
            var c=F.Search(Env(),Engine(),Limits(),Initial(),Settings(),new CancellationToken(true));
            Assert.Equal(P.SearchStatus.Cancelled,c.Status);
            Assert.Equal(0,c.Attempts);
            var a=F.Evaluate(Env(),Engine(),Limits(),Initial(500,-100),Settings(),new F.Profile(0,.5),10000);
            Assert.False(a.Result.Feasible);
        }
        [Fact]
        public void UnknownAuthorityAndUnsafeContactAreNotFeasible()
        {
            var a=F.Evaluate(Env(),new D.Propulsion(false),Limits(),Initial(100,-5),Settings(),new F.Profile(0,.5),10000);
            Assert.Equal(D.Outcome.UnknownCapability,a.Result.Outcome);
            var b=F.Evaluate(Env(),Engine(),Limits(),Initial(.1,-20),Settings(),new F.Profile(0,.5),10000);
            Assert.Equal(D.Outcome.ContactUnsafe,b.Result.Outcome);
            Assert.False(b.Result.Feasible);
        }
        [Fact]
        public void CoastDurationGridReportsFeasibleOrMissRatherThanGlobalInfeasibility()
        {
            var a=F.Search(Env(),Engine(),Limits(),Initial(),Settings());
            Assert.Equal(P.SearchStatus.SampledTargetFeasible,a.Status);
            Assert.NotNull(a.Best);
            Assert.True(a.Feasible>=9);
            Assert.True(a.Attempts>=9);
            var b=F.Search(Env(),Engine(),Limits(),Initial(),Settings(target:Y));
            Assert.Equal(P.SearchStatus.SampledFeasibleMiss,b.Status);
            Assert.InRange(b.Best.Result.Miss,502654,502656);
        }
        [Fact]
        public void LongVerticalDescentDoesNotHoldFifteenMetersPerSecond()
        {
            // Synthetic fixture inspired by the observed 6 km clearance /
            // 15 m/s descent. Mass, Isp, flow and reserve are deliberately
            // synthetic; not a reconstruction of the user's flight.
            var env=Env(301363211975.0978);
            var engine=new D.Propulsion(true,new D.Engine(350,1,9.80665,default,new D.Curve(new D.Knot(0,390))));
            var limits=new D.Limits(184000,Math.PI/30,.1,.1,.5);
            var initial=new D.State(0,X*326080,X*(-15),220000,X);
            var t=new T.Settings(.2,.5,2.94/9.81,0,
                st=>new T.ControllerAuthority(350*9.80665*390/st.Mass,350*9.80665*390/st.Mass,1,false),
                (st,dir)=>0,(ut,pos)=>320000);
            var p=new P.Settings(X,400,0,10,t,new T.Memory(T.Phase.FinalDescent,T.Mode.Off,0,0,0,0),
                intervals:1,refine:0,maxSteps:20000,safeVertical:1,safeHorizontal:1);
            var f=new F.Settings(p,320000,minPowered:100,maxPowered:100,controlTick:.5,
                coastIntervals:1,durationIntervals:1,refinements:0);
            var result=F.Evaluate(env,engine,limits,initial,f,new F.Profile(0,100),20000,true);
            Assert.Equal(D.Outcome.ContactWithinLimits,result.Result.Outcome);
            Assert.InRange(result.Result.End.UT,95,150);
            Assert.True(result.Result.End.Mass>184000);
            Assert.True(result.Reference.Any(pt=>env.Surface(pt.State.R,pt.State.Velocity).Length>30));
            Assert.True(result.Result.End.UT<6080/15.0);
        }

        private static Tracker ContactTracker()
        {
            var env=Env();var p=Engine();var lim=Limits();var s=Initial();
            var set=Settings(cap:1);
            var a=F.Evaluate(env,p,lim,s,set,new F.Profile(0,.5),10000,true);
            Assert.True(F.Validate(env,p,lim,s,set,a,10000,.01,.01,out var reference,out _));
            return new Tracker(env,p,lim,set,reference);
        }

        [Fact]
        public void TrackerInterpolatesAtActualEpochAndRejectsExpiredReference()
        {
            var t=ContactTracker();
            Assert.True(t.ReferenceAt(.05,out var state,out var cmd));
            Assert.InRange(state.R.X,320000.50499,320000.50501);
            Assert.Equal(.05,state.UT);
            Assert.True(cmd.Valid);
            Assert.False(t.ReferenceAt(-.01,out _,out _));
            Assert.False(t.ReferenceAt(t.EndUT,out _,out _));
            Assert.False(t.ReferenceAt(double.NaN,out _,out _));
        }

        [Fact]
        public void TrackerPreservesCoastAndRejectsInvalidInputOrReserve()
        {
            var t=ContactTracker();
            var s=Initial();
            var d=t.Track(s,.53,4,1,0,0);
            Assert.True(d.Valid);
            Assert.Equal(0,d.Command.Throttle);
            Assert.Equal(0,d.PositionError,8);
            Assert.False(t.Track(s,double.NaN,4,1,0,0).Valid);
            Assert.False(t.Track(s,.53,0,1,0,0).Valid);
            Assert.Equal("propellant reserve reached",
                t.Track(new D.State(0,s.R,s.Velocity,100,X),.53,4,1,0,0).Reason);
            Assert.Equal("throttle authority reduced",t.Track(s,.53,4,.8,0,0).Reason);
        }

        [Fact]
        public void TrackerRejectsDeviationAndInsufficientClearance()
        {
            var t=ContactTracker();
            var s=Initial();
            var fast=new D.State(0,s.R,X*(-10),1000,X);
            Assert.Equal("trajectory deviation",t.Track(fast,.53,4,1,0,0).Reason);
            var displaced=new D.State(0,s.R+Y*4,s.Velocity,1000,X);
            Assert.Equal("trajectory deviation",t.Track(displaced,.53,4,1,0,0).Reason);
        }

        private static Tracker PoweredTracker()
        {
            // Synthetic command history tests the tracker contract, not
            // contact feasibility or a physical replay of these two states.
            var a=new D.State(0,X*321000,X*(-20),1000,X);
            var b=new D.State(1,X*320000,X*(-.5),999,X);
            var command=new D.Command(.5,X);
            var points=new System.Collections.Generic.List<F.Point>{new F.Point(a,command,1)};
            var commands=new System.Collections.Generic.List<R.Segment>{new R.Segment(0,1,command)};
            var result=new P.Candidate(new P.Profile(0,0,new D.Command(0,X)),D.Outcome.ContactWithinLimits,
                b,0,0,0,0,1);
            var plan=new F.Candidate(new F.Profile(0,.5),result,0,0,0,points,commands);
            return new Tracker(Env(),Engine(),Limits(),Settings(cap:1),plan);
        }

        [Fact]
        public void TrackerConvertsModelThrustToLiveAccelerationAndGatesAlignment()
        {
            var t=PoweredTracker();
            Assert.True(t.ReferenceAt(0,out var s,out _));
            var a=t.Track(s,1000,8,1,0,0);
            Assert.True(a.Valid);
            Assert.Equal(.5*9.80665*400/1000/8,a.Command.Throttle,9);
            var turned=new D.State(0,s.R,s.Velocity,s.Mass,Y);
            var b=t.Track(turned,1000,8,1,0,0);
            Assert.True(b.Valid);
            Assert.Equal(0,b.Command.Throttle);
            Assert.Equal(X.X,b.Command.Direction.X,9);
        }

        [Fact]
        public void TrackerRejectsRaisedTerrainAndAxialStoppingBound()
        {
            var t=PoweredTracker();
            Assert.True(t.ReferenceAt(0,out var s,out _));
            Assert.Equal("terrain clearance below trajectory",t.Track(s,800,8,1,0,0).Reason);
            Assert.Equal("insufficient radial stopping distance",t.Track(s,1000,3.01,1,3,0).Reason);
        }

        [Fact]
        public void TrackerRejectsMalformedPlanHistory()
        {
            var t=PoweredTracker();
            var p=t.Plan;
            var bad=new F.Candidate(p.Profile,p.Result,0,0,0,
                new System.Collections.Generic.List<F.Point>{p.Reference[0]},
                new System.Collections.Generic.List<R.Segment>{new R.Segment(0,.9,p.Reference[0].Command)});
            Assert.Throws<ArgumentException>(()=>new Tracker(Env(),Engine(),Limits(),Settings(cap:1),bad));
        }

        [Fact]
        public void TrackerExecutesCompleteNominalSixKilometerTrajectory()
        {
            var env=Env(301363211975.0978);
            var engine=new D.Propulsion(true,new D.Engine(350,1,9.80665,default,new D.Curve(new D.Knot(0,390))));
            var lim=new D.Limits(184000,Math.PI/30,.1,.1,.5);
            var initial=new D.State(0,X*326080,X*(-15),220000,X);
            var terminal=new T.Settings(.2,.5,2.94/9.81,0,
                st=>new T.ControllerAuthority(350*9.80665*390/st.Mass,350*9.80665*390/st.Mass,1,false),
                (st,dir)=>0,(ut,pos)=>320000);
            var common=new P.Settings(X,400,0,10,terminal,new T.Memory(T.Phase.FinalDescent,T.Mode.Off,0,0,0,0),
                intervals:1,refine:0,maxSteps:20000,safeVertical:1,safeHorizontal:1);
            var set=new F.Settings(common,320000,minPowered:100,maxPowered:100,controlTick:.5,
                coastIntervals:1,durationIntervals:1,refinements:0);
            var plan=F.Evaluate(env,engine,lim,initial,set,new F.Profile(0,100),20000,true);
            Assert.True(plan.Result.Feasible);
            var tracker=new Tracker(env,engine,lim,set,plan);
            var actual=initial;int count=0;D.Outcome outcome=D.Outcome.Advanced;
            foreach(var point in plan.Reference)
            {
                Assert.True(env.Air(actual.UT,actual.R,out double pressure,out _));
                double raw=engine.At(pressure).Force/actual.Mass;
                double gravity=env.Mu/(actual.R.Length*actual.R.Length);
                var d=tracker.Track(actual,env.PhysicalClearance(actual),raw,1,gravity,.1);
                Assert.True(d.Valid,d.Reason+" at "+actual.UT);
                Assert.InRange(d.Command.Throttle,0,1);
                while(actual.UT<point.ValidUntil)
                {
                    double dt=Math.Min(lim.Step,point.ValidUntil-actual.UT);
                    var r=D.Advance(env,engine,lim,actual,d.Command,dt,common.SafeVertical,
                        common.SafeHorizontal,common.SafeAngle,common.SafeRate);
                    actual=r.End;outcome=r.Status;count++;
                    if(outcome!=D.Outcome.Advanced)break;
                }
                if(outcome!=D.Outcome.Advanced)break;
            }
            Assert.Equal(D.Outcome.ContactWithinLimits,outcome);
            Assert.InRange(actual.UT,95,150);
            Assert.True(actual.Mass>lim.ReserveMass);
            Assert.True(count>100);
        }

        [Fact]
        public void ExecutionPreparationCoastIsAnExplicitSearchConstraint()
        {
            var original=Settings(cap:1);
            var set=new F.Settings(original.Common,320000,approachHeight:15,approachSpeed:.5,
                minPowered:.5,maxPowered:1,controlTick:.2,coastIntervals:2,durationIntervals:2,minCoast:.2);
            Assert.True(set.Valid);
            var invalid=F.Evaluate(Env(),Engine(),Limits(),Initial(),set,new F.Profile(0,.5),1000);
            Assert.Equal(D.Outcome.InvalidInput,invalid.Result.Outcome);
            var report=F.Search(Env(),Engine(),Limits(),Initial(),set);
            Assert.NotNull(report.Best);
            Assert.True(report.Best.Profile.Coast>=.2);
        }

        private static Replanning.Request ReplanRequest(double altitude=.53,double switchUT=.2,int generation=7,
            double reserve=100,D.V? target=null)
        {
            var env=Env();var propulsion=Engine();var limits=Limits();var initial=Initial();
            var settings=Settings(cap:1,target:target);
            var candidate=F.Evaluate(env,propulsion,limits,initial,settings,new F.Profile(0,.5),10000,true);
            Assert.True(candidate.Result.Feasible);
            var old=new Tracker(env,propulsion,limits,settings,candidate);
            return new Replanning.Request(generation,Initial(altitude),switchUT,1,env,propulsion,Limits(reserve),settings,old);
        }

        [Fact]
        public void ReplanPrefixUsesMeasuredStateRatherThanFutureReference()
        {
            var request=ReplanRequest(altitude:.55);
            Assert.True(Replanning.Project(request,out var anchor,out string reason),reason);
            Assert.Equal(.2,anchor.UT);
            Assert.True(request.Previous.ReferenceAt(.2,out var nominal,out _));
            Assert.InRange((anchor.R-nominal.R).Length,.01999,.02001);
            Assert.Equal(request.Observed.Mass,anchor.Mass,8);
        }

        [Fact]
        public void PrefixRejectsInvalidEpochReserveAndCancellation()
        {
            Assert.False(Replanning.Project(ReplanRequest(switchUT:2),out _,out _));
            Assert.False(Replanning.Project(ReplanRequest(reserve:99),out _,out _));
            Assert.False(Replanning.Project(ReplanRequest(),out _,out string reason,new CancellationToken(true)));
            Assert.Equal("cancelled",reason);
            var result=Replanning.Solve(ReplanRequest(),new CancellationToken(true));
            Assert.Null(result.Tracker);
        }

        [Fact]
        public void WarmStartIsEvaluatedBeforeLatticeWhenBudgetIsOneCandidate()
        {
            var settings=Settings(cap:1,candidates:1);
            var result=F.Search(Env(),Engine(),Limits(),Initial(),settings,default,new F.Profile(.2,.75));
            Assert.Equal(P.SearchStatus.BudgetExhausted,result.Status);
            Assert.Equal(1,result.Attempts);
            Assert.NotNull(result.Best);
            Assert.Equal(.2,result.Best.Profile.Coast);
            Assert.Equal(.75,result.Best.Profile.Duration);
        }

        [Fact]
        public void ReplanPublishesReplayedReplacementAtScheduledEpoch()
        {
            var r=ReplanRequest(altitude:.55);
            var result=Replanning.Solve(r);
            Assert.NotNull(result.Tracker);
            Assert.Equal("target candidate",result.Reason);
            Assert.Equal(r.SwitchUT,result.Tracker.StartUT);
            Assert.Equal(r.Limits.ReserveMass,result.Tracker.ReserveMass);
            Assert.True(result.Tracker.Plan.Result.Feasible);
            Assert.Null(Replanning.Admission(result,r.Generation,result.Anchor,
                r.Environment.PhysicalClearance(result.Anchor),4,1,0,0,true,1,r.Limits.ReserveMass));
        }

        [Fact]
        public void AdmissionRejectsStaleLateAndDiscontinuousReplacement()
        {
            var r=ReplanRequest();var result=Replanning.Solve(r);
            Assert.NotNull(result.Tracker);
            var state=result.Anchor;
            string Check(int generation,D.State s,double reserve=100)=>
                Replanning.Admission(result,generation,s,.5,4,1,0,0,true,1,reserve);
            Assert.Equal("stale or absent candidate",Check(r.Generation+1,state));
            Assert.Equal("handoff epoch missed",Check(r.Generation,r.Observed));
            Assert.Equal("handoff epoch missed",Check(r.Generation,new D.State(.3,state.R,state.Velocity,state.Mass,state.Forward)));
            Assert.Equal("handoff state differs from projected anchor",Check(r.Generation,
                new D.State(state.UT,state.R+Y*2,state.Velocity,state.Mass,state.Forward)));
            Assert.Equal("handoff state differs from projected anchor",Check(r.Generation,
                new D.State(state.UT,state.R,state.Velocity,state.Mass,Y)));
            Assert.Equal("candidate reserve or epoch mismatch",Check(r.Generation,state,101));
        }

        [Fact]
        public void OffTargetReplacementCannotDisplaceValidTargetTrajectory()
        {
            var r=ReplanRequest(target:Y);var result=Replanning.Solve(r);
            Assert.NotNull(result.Tracker);
            Assert.Equal("degraded candidate",result.Reason);
            double clearance=r.Environment.PhysicalClearance(result.Anchor);
            Assert.Equal("retain current plan over degraded candidate",
                Replanning.Admission(result,r.Generation,result.Anchor,clearance,4,1,0,0,true,1,100));
            Assert.Null(Replanning.Admission(result,r.Generation,result.Anchor,clearance,4,1,0,0,false,1,100));
        }

        [Fact]
        public void RebaseUsesCurrentPlanEpochForRotatingTarget()
        {
            var r=ReplanRequest();var env=new D.Environment(320000,0,new D.V(0,0,.01),0,null,null,0,320000,0);
            var rotated=new Replanning.Request(r.Generation,r.Observed,.2,r.Cap,env,r.Propulsion,r.Limits,r.Settings,r.Previous);
            var anchor=new D.State(.2,r.Observed.R,r.Observed.Velocity,r.Observed.Mass,r.Observed.Forward);
            var rebased=Replanning.Rebase(rotated,anchor);
            Assert.Equal(Math.Cos(.002),rebased.Common.TargetAtInitialUT.X,10);
            Assert.Equal(Math.Sin(.002),rebased.Common.TargetAtInitialUT.Y,10);
            Assert.Equal(0,rebased.MinCoast);
        }

        [Fact]
        public void FiniteFuelAndTerminalGatePreventInventedHover()
        {
            var a=F.Evaluate(Env(2.94*320000*320000.0),Engine(),Limits(reserve:999.99),
                Initial(100,-20),Settings(cap:1,horizon:10,max:5),new F.Profile(0,5),10000);
            Assert.False(a.Result.Feasible);
            Assert.True(a.Result.Outcome==D.Outcome.ReserveExhausted||a.Result.Outcome==D.Outcome.ContactUnsafe||
                a.Result.Outcome==D.Outcome.Horizon);
        }
    }
}
