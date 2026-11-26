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
using Fallback=MuMech.Landing.AtmosphericFallbackPlanner;
using Strategic=MuMech.Landing.AtmosphericStrategicPlanner;
using Recovery=MuMech.Landing.AtmosphericRecoverability;
using Uncertainty=MuMech.Landing.AtmosphericUncertaintyReplay;
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
            Assert.InRange(reference.Result.End.UT,1.05999,1.06+D.ContactResolutionSeconds);
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
            // A high-altitude no-drag radial bound is not an independent
            // proof of imminent impact. Keep the validated plan here.
            Assert.True(t.Track(s,1000,3.01,1,3,0).Valid);
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
        public void FallbackSitesFollowSurfaceMotionAndRemainUnitRadials()
        {
            var env=Env();
            var s=new D.State(0,X*321000,X*(-20)+Y*100,1000,X);
            var sites=Fallback.SiteRadials(env,s,5,5);
            Assert.Equal(5,sites.Length);
            Assert.All(sites,radial=>Assert.InRange(radial.Length,.999999,1.000001));
            Assert.True(sites[0].Y>0);
            Assert.True(sites[2].Y>sites[0].Y);
            Assert.Equal(0,sites[4].Y);
            Assert.Single(Fallback.SiteRadials(env,Initial(),5,5));
            Assert.Empty(Fallback.SiteRadials(env,s,0,5));
        }

        [Fact]
        public void SiteEnvironmentUsesOwnTerrainHeightAndRetainsOriginalPhysics()
        {
            var env=Env(301363211975.0978);
            var site=new Fallback.Site(X,320250);
            var alternative=Fallback.SiteEnvironment(env,site);
            Assert.NotNull(alternative);
            Assert.Equal(env.Mu,alternative.Mu);
            Assert.Equal(env.Radius,alternative.Radius);
            Assert.Equal(750,alternative.PhysicalClearance(Initial(1000)),7);
            Assert.Equal(1000,env.PhysicalClearance(Initial(1000)),7);
            Assert.Null(Fallback.SiteEnvironment(env,new Fallback.Site(Y,-1)));
        }

        [Fact]
        public void FallbackSearchReplaysDifferentSiteWithoutRelabellingOriginalTarget()
        {
            var request=ReplanRequest(target:Y);
            var result=Fallback.Solve(request,new[]{new Fallback.Site(X,320000)});
            Assert.NotNull(result.Tracker);
            Assert.Equal("local-site candidate",result.Reason);
            Assert.Equal(request.SwitchUT,result.Tracker.StartUT);
            Assert.Equal(request.Limits.ReserveMass,result.Tracker.ReserveMass);
            Assert.InRange(result.Tracker.TargetMiss,0,1);
            Assert.InRange(result.OriginalMiss,502654,502656);
            Assert.Equal(D.Outcome.ContactWithinLimits,result.Tracker.Plan.Result.Outcome);
            Assert.True(result.Tracker.Plan.Result.End.Mass>request.Limits.ReserveMass);
        }

        [Fact]
        public void FallbackRejectsUnknownSiteImpossibleTerrainAndCancelledWork()
        {
            var request=ReplanRequest();
            Assert.Null(Fallback.Solve(request,Array.Empty<Fallback.Site>()).Tracker);
            Assert.Null(Fallback.Solve(request,new[]{new Fallback.Site(X,320100)}).Tracker);
            Assert.Null(Fallback.Solve(request,new[]{new Fallback.Site(X,320000)},
                new CancellationToken(true)).Tracker);
        }

        [Fact]
        public void FallbackAdmissionChecksEpochGenerationStateAndOriginalReserve()
        {
            var request=ReplanRequest(target:Y);
            var result=Fallback.Solve(request,new[]{new Fallback.Site(X,320000)});
            Assert.NotNull(result.Tracker);
            Assert.True(result.Tracker.ReferenceAt(result.SwitchUT,out var state,out _));
            string Check(int generation,D.State s,double reserve=100)=>
                Fallback.Admission(result,generation,s,result.Environment.PhysicalClearance(s),4,1,0,0,reserve);
            Assert.Null(Check(request.Generation,state));
            Assert.Equal("stale fallback",Check(request.Generation+1,state));
            Assert.Equal("fallback activation window missed",Check(request.Generation,request.Observed));
            Assert.Equal("fallback activation window missed",Check(request.Generation,
                new D.State(state.UT+3,state.R,state.Velocity,state.Mass,state.Forward)));
            Assert.Equal("fallback state mismatch",Check(request.Generation,
                new D.State(state.UT,state.R,state.Velocity,state.Mass,Y)));
            Assert.Equal("fallback reserve or epoch mismatch",Check(request.Generation,state,101));
            Assert.True(result.Tracker.ReferenceAt(result.SwitchUT+.1,out var later,out _));
            Assert.Null(Check(request.Generation,later));
        }

        private static D.State Circular(D.Environment env,double altitude=100000)
        {
            double radius=env.Radius+altitude;
            return new D.State(0,X*radius,Y*Math.Sqrt(env.Mu/radius),1000,Y*(-1));
        }

        // Representative KSP flight conditions exercise the production
        // admission/settings code, not an alternate test-only policy.
        // Duna radius/mu/rotation from previous captured body data; the
        // craft mass, drag faces and sampled air curve are synthetic until
        // the matching raw KSP snapshot is supplied.
        private static D.Environment RepresentativeDuna(double area)
        {
            var p=new D.Curve(new D.Knot(0,20000),new D.Knot(10000,4500),
                new D.Knot(20000,1000),new D.Knot(30000,150),new D.Knot(40000,20),new D.Knot(50000,0));
            var rho=new D.Curve(new D.Knot(0,.145),new D.Knot(10000,.035),
                new D.Knot(20000,.008),new D.Knot(30000,.0012),new D.Knot(40000,.00016),new D.Knot(50000,0));
            return new D.Environment(320000,301363211975.0978,new D.V(0,0,-9.5900344839e-5),
                50000,p,rho,area,320000,2);
        }
        [Fact]
        public void KspAdmissionVacuumDragAndDefaultParachuteOption()
        {
            var drag=new MuMech.Landing.DunaLandingAdmission.DragAccumulator();
            // Valid prospective cube faces, even when current AreaDrag=0
            // and atmospheric pressure and density are both zero.
            foreach(double face in new[]{2.0,2,4,4,3,3})drag.AddFace(.8,face);
            Assert.Equal(2.4,drag.Finish(1,1),8);
            var env=RepresentativeDuna(drag.Finish(1,1));
            var orbit=Circular(env,60000);
            Assert.True(env.Air(orbit.UT,orbit.R,out double p,out double rho));
            Assert.Equal(0,p);Assert.Equal(0,rho);
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Mode.Orbit,
                MuMech.Landing.DunaLandingAdmission.Choose(env,orbit,0,0,out var reason));
            Assert.Null(reason);
            Assert.Null(MuMech.Landing.DunaLandingAdmission.CheckContext(true,false,false,1,true,
                true,0,false,false,false,false,false,false,30));
            Assert.Equal("parachute dynamics unsupported",
                MuMech.Landing.DunaLandingAdmission.CheckContext(true,false,false,1,true,
                true,1,false,false,false,false,false,false,30));
            Assert.Equal("active RCS unsupported",
                MuMech.Landing.DunaLandingAdmission.CheckContext(true,false,false,1,true,
                true,0,false,true,false,false,false,false,30));
            Assert.Equal("active unpacked flight at 1x required",
                MuMech.Landing.DunaLandingAdmission.CheckContext(true,false,false,5,true,
                true,0,false,false,false,false,false,false,30));
            drag.AddFace(double.NaN,2);
            Assert.True(double.IsNaN(drag.Finish(1,1)));
            Assert.True(double.IsNaN(new MuMech.Landing.DunaLandingAdmission.DragAccumulator().Finish(1,1)));
            var noDrag=RepresentativeDuna(0);
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Mode.Refused,
                MuMech.Landing.DunaLandingAdmission.Choose(noDrag,orbit,0,0,out reason));
            Assert.Equal("prospective drag area unavailable",reason);
        }

        [Fact]
        public void KspAdmissionEntryAndResourceAuthority()
        {
            var env=RepresentativeDuna(3);
            var descending=new D.State(31154879,X*360000,new D.V(-50,500,0),1000,X);
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Mode.Descent,
                MuMech.Landing.DunaLandingAdmission.Choose(env,descending,0,-50,out var reason));
            Assert.Null(reason);
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Mode.Refused,
                MuMech.Landing.DunaLandingAdmission.Choose(env,descending,.2,-50,out reason));
            Assert.Equal("engagement requires unpowered coast",reason);
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Mode.Refused,
                MuMech.Landing.DunaLandingAdmission.Choose(env,descending,0,5,out reason));
            var inventory=new System.Collections.Generic.Dictionary<int,double>{{1,90},{2,110}};
            var mixture=new System.Collections.Generic.Dictionary<int,double>{{1,.45},{2,.55}};
            Assert.Equal(1000,MuMech.Landing.DunaLandingAdmission.UsableFuel(inventory,mixture,id=>5,2000),8);
            inventory[2]=0;
            Assert.True(double.IsNaN(MuMech.Landing.DunaLandingAdmission.UsableFuel(inventory,mixture,id=>5,2000)));
            Assert.True(double.IsNaN(MuMech.Landing.DunaLandingAdmission.ImpulseDerate(0,2000,10000)));
            Assert.Equal(.9,MuMech.Landing.DunaLandingAdmission.ImpulseDerate(5,2000,10000),8);
            Assert.Null(MuMech.Landing.DunaLandingAdmission.Settings(env,Engine(),descending,X,0,5,.9,.5));
            Assert.NotNull(MuMech.Landing.DunaLandingAdmission.Settings(env,Engine(),descending,X,1,5,.9,.5));
        }

        [Fact]
        public void SingleClickAdmissionNeverEngagesStaleOrUnvalidatedPlan()
        {
            Assert.True(MuMech.Landing.DunaLandingAdmission.UsesIntegratedTargeted("Duna",true,true));
            Assert.False(MuMech.Landing.DunaLandingAdmission.UsesIntegratedTargeted("Mun",false,true));
            Assert.False(MuMech.Landing.DunaLandingAdmission.UsesIntegratedTargeted("Duna",true,false));
            Assert.True(MuMech.Landing.DunaLandingAdmission.OwnsActuators(0,false,1,true,false,false));
            Assert.True(MuMech.Landing.DunaLandingAdmission.OwnsActuators(1,true,2,true,true,true));
            Assert.False(MuMech.Landing.DunaLandingAdmission.OwnsActuators(2,true,2,true,true,true));
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Progress.Waiting,
                MuMech.Landing.DunaLandingAdmission.Next(true,true,true,false,false,false));
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Progress.Waiting,
                MuMech.Landing.DunaLandingAdmission.Next(true,true,false,false,false,false));
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Progress.Engage,
                MuMech.Landing.DunaLandingAdmission.Next(true,true,false,true,true,true));
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Progress.Refuse,
                MuMech.Landing.DunaLandingAdmission.Next(true,true,false,true,true,false));
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Progress.Refuse,
                MuMech.Landing.DunaLandingAdmission.Next(false,true,false,true,true,true));
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Progress.Refuse,
                MuMech.Landing.DunaLandingAdmission.Next(true,false,false,true,true,true));
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Progress.Refuse,
                MuMech.Landing.DunaLandingAdmission.Next(true,true,false,false,true,false));
        }

        [Fact]
        public void RepresentativeDunaOrbitToTouchdownThroughProductionAdmission()
        {
            // Fixed target set before search. Enough propellant and thrust
            // for a representative small rocket-powered Duna lander.
            var env=RepresentativeDuna(3);
            var initial=Circular(env,60000);
            var engine=Engine(4);
            var lim=new D.Limits(420,Math.PI/30,.1,.1,.5);
            var target=new D.V(Math.Cos(2.3),Math.Sin(2.3),0);
            var mode=MuMech.Landing.DunaLandingAdmission.Choose(env,initial,0,0,out string reason);
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Mode.Orbit,mode);
            var set=MuMech.Landing.DunaLandingAdmission.Settings(env,engine,initial,target,1,
                4*400*9.80665/initial.Mass, .9,.5);
            Assert.NotNull(set);
            var strategic=new Strategic.Settings(set,preparation:45,engagementLead:45);
            var diagnostics=new System.Collections.Generic.List<string>();
            var report=Strategic.Search(env,engine,lim,initial,strategic,diagnostic:diagnostics.Add);
            string details="orbital-admission-fixture status="+report.Status+" prefixes="+report.Prefixes+
                " approaches="+report.Approaches+" steps="+report.Steps+" feasible="+report.Feasible+
                " unresolved="+report.Unresolved+" miss="+report.Best?.OriginalMiss+"\n"+string.Join("\n",diagnostics);
            Console.WriteLine(details);
            Assert.True(report.Best!=null,details);
            Assert.True(report.Best.ReplayPassed);
            Assert.True(report.Scenarios.Passed);
            Assert.InRange(report.Best.OriginalMiss,0,set.Common.TargetTolerance);
            var tracker=new Tracker(env,engine,lim,set,report.Best.Combined);
            var decision=tracker.Track(initial,env.PhysicalClearance(initial),4*400*9.80665/initial.Mass,
                1,env.Mu/(initial.R.Length*initial.R.Length),.1);
            Assert.True(decision.Valid,decision.Reason);
            Assert.Equal(0,decision.Command.Throttle);
            Assert.Equal(MuMech.Landing.DunaLandingAdmission.Progress.Engage,
                MuMech.Landing.DunaLandingAdmission.Next(true,true,false,true,true,decision.Valid));
        }

        [Fact]
        public void StrategicCircularOrbitAndPeriapsisSeedMatchAnalyticGeometry()
        {
            var env=Env(301363211975.0978);
            var state=Circular(env);
            Assert.True(Strategic.TryOrbit(env,state,out var orbit));
            Assert.Equal(420000,orbit.SemiMajor,6);
            Assert.InRange(orbit.Eccentricity,0,1e-10);
            Assert.Equal(420000,orbit.Periapsis,6);
            Assert.Equal(2*Math.PI*Math.Sqrt(Math.Pow(420000.0,3)/env.Mu),orbit.Period,8);
            double delta=Strategic.SeedDeltaV(env,state,300000);
            var changed=new D.State(0,state.R,state.Velocity-Y*delta,state.Mass,state.Forward);
            Assert.True(Strategic.TryOrbit(env,changed,out var transfer));
            Assert.Equal(300000,transfer.Periapsis,6);
            Assert.False(Strategic.TryOrbit(env,new D.State(0,state.R,Y*2000,1000,X),out _));
        }

        [Fact]
        public void FiniteDeorbitBurnChangesPlaneAndConsumesMassBeforeHandoff()
        {
            var env=Env(301363211975.0978);var state=Circular(env);
            var limits=Limits(step:.125);
            var settings=new Strategic.Settings(Settings(cap:1),preparation:1,maxWait:10,maxBurn:40);
            var retro=Strategic.EvaluatePrefix(env,Engine(),limits,state,settings,
                new Strategic.Burn(1,25,0),10000,true);
            var tilted=Strategic.EvaluatePrefix(env,Engine(),limits,state,settings,
                new Strategic.Burn(1,25,.2),10000,true);
            Assert.Equal(D.Outcome.Advanced,retro.Outcome);
            Assert.Equal(D.Outcome.Advanced,tilted.Outcome);
            Assert.True(Strategic.TryOrbit(env,tilted.End,out var orbit));
            Assert.True(orbit.Periapsis<env.Radius);
            Assert.True(Math.Abs(tilted.End.Velocity.Z)>10);
            Assert.InRange(Math.Abs(retro.End.Velocity.Z),0,1e-8);
            Assert.Equal(975,tilted.End.Mass,7);
            Assert.Equal(26.5,tilted.End.UT,8);
            Assert.Equal(1,tilted.FirstPoweredUT,8);
            Assert.Equal(tilted.Points.Count,tilted.Commands.Count);
            Assert.Equal(tilted.End.UT,tilted.Commands.Last().EndUT);
            for(int i=1;i<tilted.Commands.Count;i++)
                Assert.Equal(tilted.Commands[i-1].EndUT,tilted.Commands[i].StartUT);
        }

        [Fact]
        public void StrategicEngagementLeadIsAnUnpoweredUnturnedInterval()
        {
            var env=Env(301363211975.0978);var state=Circular(env);
            var settings=new Strategic.Settings(Settings(cap:1),preparation:1,maxWait:10,maxBurn:40,engagementLead:3);
            var limits=Limits(step:.125);
            Assert.Equal(D.Outcome.InvalidInput,Strategic.EvaluatePrefix(env,Engine(),limits,state,settings,
                new Strategic.Burn(3,25,0),10000).Outcome);
            var prefix=Strategic.EvaluatePrefix(env,Engine(),limits,state,settings,
                new Strategic.Burn(4,25,.2),10000,true);
            Assert.Equal(D.Outcome.Advanced,prefix.Outcome);
            Assert.Equal(4,prefix.FirstPoweredUT);
            Assert.All(prefix.Points.Where(p=>p.State.UT<3),p=>{
                Assert.Equal(0,p.Command.Throttle);
                Assert.InRange((p.Command.Direction-state.Forward).Length,0,1e-12);
            });
        }

        [Fact]
        public void StrategicPrefixRejectsReserveExhaustionAndBudgetOrCancellation()
        {
            var env=Env(301363211975.0978);var state=Circular(env);
            var settings=new Strategic.Settings(Settings(cap:1),preparation:1,maxWait:10,maxBurn:40);
            var burn=new Strategic.Burn(1,25,0);
            Assert.Equal(D.Outcome.ReserveExhausted,Strategic.EvaluatePrefix(env,Engine(),
                Limits(reserve:990,step:.125),state,settings,burn,10000).Outcome);
            Assert.Equal(D.Outcome.Horizon,Strategic.EvaluatePrefix(env,Engine(),
                Limits(step:.125),state,settings,burn,1).Outcome);
            Assert.Equal(D.Outcome.Cancelled,Strategic.EvaluatePrefix(env,Engine(),
                Limits(step:.125),state,settings,burn,10000,token:new CancellationToken(true)).Outcome);
            Assert.Equal(D.Outcome.InvalidInput,Strategic.EvaluatePrefix(env,Engine(),
                Limits(step:.125),state,settings,new Strategic.Burn(0,25,0),10000).Outcome);
        }

        [Fact]
        public void CombinedHistoryReplaysFromOriginalStateAndRejectsDiscontinuousJoin()
        {
            // Small deterministic prefix/terminal fixture tests the whole
            // history contract independently of orbital seed selection.
            var env=Env();var engine=Engine();var limits=Limits(step:.125);
            var initial=Initial();var command=new D.Command(0,X);
            var step=D.Advance(env,engine,limits,initial,command,.125);
            Assert.Equal(D.Outcome.Advanced,step.Status);
            var prefix=new Strategic.Prefix(new Strategic.Burn(0,.125,0),step.End,D.Outcome.Advanced,1,
                step.MinimumClearance,double.NaN,.125,
                new System.Collections.Generic.List<F.Point>{new F.Point(initial,command,.125)},
                new System.Collections.Generic.List<R.Segment>{new R.Segment(0,.125,command)});
            var set=Settings(cap:1);
            var approach=F.Evaluate(env,engine,limits,step.End,set,new F.Profile(0,.5),10000,true);
            Assert.True(approach.Result.Feasible);
            var settings=new Strategic.Settings(set);
            var combined=Strategic.Combine(env,engine,limits,initial,settings,prefix,set,approach,10000,out int replaySteps);
            Assert.NotNull(combined);
            Assert.True(combined.ReplayPassed);
            Assert.True(replaySteps>1);
            Assert.Equal(initial.UT,combined.Combined.Reference[0].State.UT);
            Assert.Equal(step.End.UT,combined.HandoffUT);
            Assert.Equal(approach.Result.End.Mass,combined.Combined.Result.End.Mass);
            Assert.Equal(approach.Result.End.UT,combined.Combined.Result.End.UT);
            Assert.Equal(0,combined.OriginalMiss,6);
            Assert.NotNull(new Tracker(env,engine,limits,set,combined.Combined));
            var wrong=F.Evaluate(env,engine,limits,Initial(.6),set,new F.Profile(0,.5),10000,true);
            Assert.Null(Strategic.Combine(env,engine,limits,initial,settings,prefix,set,wrong,10000,out _));
            Assert.Null(Strategic.Combine(env,engine,limits,initial,settings,prefix,set,approach,1,out _));
        }

        [Fact]
        public void StrategicSearchDoesNotClaimInfeasibilityOnIncompleteSearch()
        {
            var env=Env(301363211975.0978);
            var settings=new Strategic.Settings(Settings(cap:1),preparation:1,maxWait:10,maxSteps:1);
            var report=Strategic.Search(env,Engine(),Limits(step:.125),Circular(env),settings);
            Assert.Equal(P.SearchStatus.BudgetExhausted,report.Status);
            Assert.Null(report.Best);
            Assert.Equal(1,report.Steps);
            Assert.Equal(P.SearchStatus.Cancelled,Strategic.Search(env,Engine(),Limits(),Circular(env),settings,
                new CancellationToken(true)).Status);
            Assert.Equal(P.SearchStatus.InvalidInput,Strategic.Search(env,Engine(),Limits(),Initial(),settings).Status);
        }

        [Fact]
        public void StrategicHandoffRebasesRotatingTargetAndKeepsAbsoluteFuelAccounting()
        {
            var env=new D.Environment(320000,301363211975.0978,new D.V(0,0,.0001),0,null,null,0,320000,0);
            var initial=Circular(env);var state=new D.State(100,initial.R,initial.Velocity,900,X);
            var settings=new Strategic.Settings(Settings(cap:1));
            var approach=Strategic.ApproachAt(env,Engine(),initial,state,settings);
            Assert.NotNull(approach);
            Assert.True(approach.Valid);
            Assert.Equal(Math.Cos(.01),approach.Common.TargetAtInitialUT.X,10);
            Assert.Equal(Math.Sin(.01),approach.Common.TargetAtInitialUT.Y,10);
            Assert.Equal(0,approach.MinCoast);
        }

        [Fact]
        public void IndependentRadialScreenDistinguishesWarningFromImminentHazard()
        {
            var state=new D.State(0,X*326080,X*(-15),1000,X);
            var clear=Recovery.Assess(state,state.Velocity,6080,6,1,2.94,.1,.2,1,100,.5);
            Assert.Equal(Recovery.Level.Clear,clear.Status);
            var warning=Recovery.Assess(state,state.Velocity,6080,2.95,1,2.94,.1,.2,1,100,.5);
            Assert.Equal(Recovery.Level.Warning,warning.Status);
            var hazard=Recovery.Assess(state,state.Velocity,40,3.1,1,2.94,.1,.2,1,100,.5);
            Assert.Equal(Recovery.Level.Hazard,hazard.Status);
            Assert.True(hazard.OptimisticDistance>40);
            Assert.Equal(Recovery.Level.Unknown,Recovery.Assess(state,state.Velocity,double.NaN,
                6,1,2.94,.1,.2,1,100,.5).Status);
        }

        [Fact]
        public void IndependentFuelAndTurnMarginsRemainExplicit()
        {
            var state=new D.State(0,X*321000,X*(-20),1000,Y);
            var a=Recovery.Assess(state,state.Velocity,1000,6,1,2.94,.1,1,1,999,.5);
            Assert.Equal(Recovery.Level.Warning,a.Status);
            Assert.True(a.FuelMargin<0);
            Assert.True(a.DelayedDistance>a.OptimisticDistance);
            var b=Recovery.Assess(state,state.Velocity,1000,6,1,2.94,.1,1,1,1000,.5);
            Assert.Equal(Recovery.Level.Hazard,b.Status);
        }

        [Fact]
        public void BestEffortAllowsFastDescentAtAltitudeAndBrakesImminentHazard()
        {
            var state=new D.State(0,X*326080,X*(-15),1000,X);
            Assert.True(Recovery.TryBestEffort(state,state.Velocity,6080,6,1,2.94,.2,1,100,.5,
                false,out var coast));
            Assert.Equal(0,coast.Throttle);
            Assert.True(Recovery.TryBestEffort(state,state.Velocity,40,6,1,2.94,.2,1,100,.5,
                true,out var brake));
            Assert.Equal(1,brake.Throttle);
            Assert.Equal(1,brake.Direction.X,9);
            var turned=new D.State(0,state.R,state.Velocity,1000,Y);
            Assert.True(Recovery.TryBestEffort(turned,turned.Velocity,40,6,1,2.94,.2,1,100,.5,
                true,out var align));
            Assert.Equal(0,align.Throttle);
            Assert.Equal(1,align.Direction.X,9);
        }

        [Fact]
        public void BestEffortPreservesReserveAndRejectsUnknownFlow()
        {
            var state=new D.State(0,X*320100,X*(-20),1000,X);
            Assert.True(Recovery.TryBestEffort(state,state.Velocity,20,6,1,2.94,1,10,999,.5,
                true,out var command));
            Assert.InRange(command.Throttle,0,1.0/12);
            Assert.False(Recovery.TryBestEffort(state,state.Velocity,20,6,1,2.94,1,10,1000,.5,
                true,out _));
            Assert.False(Recovery.TryBestEffort(state,state.Velocity,20,6,1,2.94,1,double.NaN,100,.5,
                true,out _));
        }

        [Fact]
        public void ScenarioScreenReportsAllSampledContactsAndBudgets()
        {
            var tracker=ContactTracker();var env=Env();var engine=Engine();var limits=Limits();
            var report=Uncertainty.Validate(env,engine,limits,tracker,10000);
            Assert.True(report.Passed,report.Reason);
            Assert.Equal(4,report.Trials.Count);
            Assert.All(report.Trials,t=>Assert.Equal(D.Outcome.ContactWithinLimits,t.Outcome));
            Assert.InRange(report.WorstMiss,0,.001);
            // Fine-step closed-loop feedback can spend a small amount of
            // propellant even in this near-contact fixture. Test the actual
            // reserve and a tight consumption bound, not zero expenditure.
            Assert.InRange(report.MinimumEndMass,999.9,1000);
            var limited=Uncertainty.Validate(env,engine,limits,tracker,1);
            Assert.False(limited.Passed);
            Assert.Equal(1,limited.Steps);
            Assert.False(Uncertainty.Validate(env,engine,limits,tracker,10000,new CancellationToken(true)).Passed);
        }

        [Fact]
        public void ScenarioImpulseScalingPreservesFlowAndExposesUnrecoverableHistory()
        {
            var engine=Engine();
            var original=engine.At(0);var weaker=engine.WithImpulseScale(.95).At(0);
            Assert.True(weaker.Known);
            Assert.Equal(original.Flow,weaker.Flow,12);
            Assert.Equal(original.Force*.95,weaker.Force,9);
            // Malformed physical trajectory is intentionally used to show
            // the independent closed-loop screen rejects a nominal metadata
            // contact claim rather than trusting it.
            var trial=Uncertainty.Run(Env(),engine,Limits(),PoweredTracker(),
                new Uncertainty.Scenario("severe force loss",.01,1,1,1),10000);
            Assert.False(trial.Passed);
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
