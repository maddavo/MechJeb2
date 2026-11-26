using System;
using System.Linq;
using System.Threading;
using Xunit;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using T=MuMech.Landing.AtmosphericTerminalReplay;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;

namespace MechJebLibTest.LandingPredictionTests
{
    public class AtmosphericFeasibleRangePlannerTests
    {
        private static D.V X=>new D.V(1,0,0);
        private static D.V Y=>new D.V(0,1,0);
        private static D.Environment Env(D.V omega=default,D.TerrainModel terrain=null)=>
            new D.Environment(320000,0,omega,0,null,null,0,320000,0,terrain:terrain);
        private static D.Propulsion Engines()=>new D.Propulsion(true,
            new D.Engine(1,1,9.80665,default,new D.Curve(new D.Knot(0,400))));
        private static D.Limits Limits()=>new D.Limits(100,10,0,0,.1);
        private static D.State Start(double altitude=.5,double vertical=-.5)=>
            new D.State(0,X*(320000+altitude),X*vertical,1000,X);
        private static T.Memory Memory()=>new T.Memory(T.Phase.FinalDescent,T.Mode.KeepVertical,0,0,0,0);
        private static T.Settings Terminal(double cap=0)=>new T.Settings(.2,1,0,0,
            s=>new T.ControllerAuthority(4,4*cap,cap,false),(s,d)=>0,(ut,p)=>320000);
        private static P.Settings Settings(D.V? target=null,double horizon=4,double maxIgnition=.6,
            int intervals=2,int refine=1,int candidates=128,int steps=10000,double cap=0,double cutoff=0,
            double tolerance=1,T.Settings terminal=null)=>
            new P.Settings(target??X,horizon,maxIgnition,tolerance,terminal??Terminal(cap),Memory(),
                intervals,refine,candidates,steps,captureClearance:0,captureSpeed:15,
                entryCutoffDelay:cutoff,commandCap:cap);
        private static D.Command Coast()=>new D.Command(0,X);

        [Fact]
        public void RotatingTargetUsesContactEpochAndSignedAxis()
        {
            var env=Env(new D.V(0,0,.01));
            var end=new D.State(Math.PI/2/.01,Y*320000,default,1000,Y);
            Assert.InRange(P.Miss(env,Start(),end,X),0,1e-8);
            Assert.Equal(Math.PI*320000,P.Miss(Env(new D.V(0,0,-.01)),Start(),end,X),7);
        }
        [Fact]
        public void RotatingContactConsistencyRemovesOnlyExpectedSurfaceRotation()
        {
            var env=Env(new D.V(0,0,-9.5900344839e-5));
            var a=new D.State(100,X*320000,default,1000,X);
            var rotated=P.RotateTarget(a.R,env.Omega,1.6);
            var b=new D.State(101.6,rotated,default,1000,rotated.Unit);
            Assert.True((a.R-b.R).Length>40);
            Assert.InRange(P.ContactPositionDifference(env,a,b),0,1e-8);
            var raised=new D.State(b.UT,b.R+b.R.Unit*2,default,1000,b.Forward);
            Assert.Equal(2,P.ContactPositionDifference(env,a,raised),7);
        }

        [Fact]
        public void SweepIncludesAllCoarseAndRefinedTimingNodes()
        {
            var r=P.Search(Env(),Engines(),Limits(),Start(),Settings(),0,Coast());
            Assert.Equal(P.SearchStatus.SampledTargetFeasible,r.Status);
            Assert.True(r.RequestedGridCompleted);
            Assert.Equal(5,r.Candidates.Count);
            Assert.Equal(5,r.FeasibleCount);
            Assert.Equal(0,r.UnresolvedCount);
            Assert.Equal(0,r.Candidates.Min(c=>c.Profile.IgnitionDelay));
            Assert.Equal(.6,r.Candidates.Max(c=>c.Profile.IgnitionDelay));
            Assert.All(r.Candidates,c=>{
                Assert.Equal(D.Outcome.ContactWithinLimits,c.Outcome);
                Assert.InRange(c.End.UT,.99999,1+D.ContactResolutionSeconds);
                Assert.Equal(1000,c.End.Mass,9);
            });
        }
        [Fact]
        public void TerminalCaptureAloneIsNotFeasible()
        {
            var r=P.Search(Env(),Engines(),Limits(),Start(50,-.5),Settings(horizon:1,maxIgnition:0),0,Coast());
            Assert.Equal(P.SearchStatus.NoDemonstratedLanding,r.Status);
            Assert.Null(r.Best);
            Assert.Equal(1,r.UnresolvedCount);
            Assert.Equal(D.Outcome.Horizon,r.Candidates[0].Outcome);
            Assert.Equal(0,r.Candidates[0].TerminalStartUT);
        }
        [Fact]
        public void UnknownTerrainAndUnsafeContactNeverRank()
        {
            var unknown=Env(terrain:new D.TerrainModel(s=>default,(a,b,c)=>double.NaN));
            var a=P.Search(unknown,Engines(),Limits(),Start(),Settings(),0,Coast());
            Assert.Null(a.Best);
            Assert.All(a.Candidates,c=>Assert.Equal(D.Outcome.UnknownTerrain,c.Outcome));
            var b=P.Search(Env(),Engines(),Limits(),Start(.5,-10),Settings(),0,Coast());
            Assert.Null(b.Best);
            Assert.All(b.Candidates,c=>Assert.Equal(D.Outcome.ContactUnsafe,c.Outcome));
        }
        [Fact]
        public void BestFeasibleMissIsNotClaimOfTargetInfeasibility()
        {
            var r=P.Search(Env(),Engines(),Limits(),Start(),Settings(target:Y),0,Coast());
            Assert.Equal(P.SearchStatus.SampledFeasibleMiss,r.Status);
            Assert.True(r.Best.HasValue);
            Assert.Equal(Math.PI/2*320000,r.Best.Value.Miss,6);
        }
        [Fact]
        public void LexicographicRankingPrefersReserveWithinTolerance()
        {
            P.Candidate C(double miss,double mass,D.Outcome outcome=D.Outcome.ContactWithinLimits)=>
                new P.Candidate(default,outcome,new D.State(1,X*320000,default,mass,X),
                    miss,0,double.NaN,0,1);
            Assert.True(P.Better(C(9,999),C(1,998),10));
            Assert.True(P.Better(C(11,998),C(12,999),10));
            Assert.True(P.Better(C(9,998),C(11,999),10));
            Assert.False(P.Better(C(0,1000,D.Outcome.ContactUnsafe),C(9,998),10));
        }
        [Fact]
        public void CandidateAndStepBudgetsAndCancellationAreExplicit()
        {
            var a=P.Search(Env(),Engines(),Limits(),Start(),Settings(candidates:2),0,Coast());
            Assert.Equal(P.SearchStatus.BudgetExhausted,a.Status);
            Assert.False(a.RequestedGridCompleted);
            Assert.Equal(2,a.Candidates.Count);
            var b=P.Search(Env(),Engines(),Limits(),Start(),Settings(steps:1),0,Coast());
            Assert.Equal(P.SearchStatus.BudgetExhausted,b.Status);
            Assert.Equal(1,b.Steps);
            Assert.Null(b.Best);
            var c=P.Search(Env(),Engines(),Limits(),Start(),Settings(),0,Coast(),new CancellationToken(true));
            Assert.Equal(P.SearchStatus.Cancelled,c.Status);
            Assert.Empty(c.Candidates);
        }
        [Fact]
        public void FiniteCorrectionRequiresExplicitCutoffAndCommandCap()
        {
            var p=new P.Profile(.4,.2,new D.Command(.5,Y));
            var a=P.Evaluate(Env(),Engines(),Limits(),Start(),Settings(cap:1),p,1000);
            Assert.Equal(D.Outcome.InvalidInput,a.Outcome);
            var b=P.Evaluate(Env(),Engines(),Limits(),Start(),Settings(cap:.2,cutoff:.3),p,1000);
            Assert.Equal(D.Outcome.InvalidInput,b.Outcome);
        }
        [Fact]
        public void CorrectionSeedsIncludeBothSignsAndNoBurn()
        {
            var s=new D.State(0,X*321000,new D.V(-1,10,0),1000,X);
            var options=P.GenerateCorrections(Env(),s,Settings(maxIgnition:3,cutoff:2,cap:1),
                new[]{.5,2.0,3.0},.8);
            Assert.Equal(21,options.Length);
            Assert.Equal(0,options[0].Seconds);
            Assert.Contains(options,c=>c.Seconds==2&&D.V.Dot(c.Command.Direction,Y)>.99);
            Assert.Contains(options,c=>c.Seconds==2&&D.V.Dot(c.Command.Direction,Y)<-.99);
            Assert.Contains(options,c=>c.Seconds==2&&c.Command.Direction.Z>.99);
            Assert.Contains(options,c=>c.Seconds==2&&c.Command.Direction.Z<-.99);
        }
        [Fact]
        public void JointSearchSamplesCorrectionAndTimingUnderOneBudget()
        {
            var opts=new[]{new P.CorrectionOption(0,Coast()),new P.CorrectionOption(.2,new D.Command(.25,Y))};
            var r=P.SearchJoint(Env(),Engines(),Limits(),Start(),Settings(cap:1,cutoff:.3,candidates:2),opts);
            Assert.Equal(P.SearchStatus.BudgetExhausted,r.Status);
            Assert.Equal(2,r.Candidates.Count);
            Assert.Equal(0,r.Candidates[0].Profile.CorrectionSeconds);
            Assert.Equal(.2,r.Candidates[1].Profile.CorrectionSeconds);
            Assert.True(r.Candidates[1].End.Mass<1000);
        }
        [Fact]
        public void JointPlannerSelectsTargetReachingCorrectionInsteadOfCoastMiss()
        {
            var s=new D.State(0,X*320000.5,new D.V(-.5,0,0),1000,Y);
            var seed=Settings(cap:.5,cutoff:.3,terminal:Terminal(0));
            var correction=new D.Command(.025,Y);
            var b=P.Evaluate(Env(),Engines(),Limits(),s,seed,
                new P.Profile(.2,.2,correction),10000);
            Assert.True(b.Feasible);
            var set=Settings(target:b.End.R.Unit,cap:.5,cutoff:.3,tolerance:1e-4,terminal:Terminal(0));
            var coast=P.Search(Env(),Engines(),Limits(),s,set,0,new D.Command(0,Y));
            var joint=P.SearchJoint(Env(),Engines(),Limits(),s,set,
                new[]{new P.CorrectionOption(0,new D.Command(0,Y)),new P.CorrectionOption(.2,correction)});
            Assert.Equal(P.SearchStatus.SampledFeasibleMiss,coast.Status);
            Assert.Equal(P.SearchStatus.SampledTargetFeasible,joint.Status);
            Assert.Equal(.2,joint.Best.Value.Profile.CorrectionSeconds);
            Assert.True(joint.Best.Value.Miss<coast.Best.Value.Miss);
            Assert.True(joint.Best.Value.End.Mass<coast.Best.Value.End.Mass);
        }

        [Fact]
        public void ActualFiniteCorrectionCanMoveTouchdownAndConsumeFuel()
        {
            // Analytic near-contact synthetic vacuum fixture, not a Duna
            // flight fit. Terminal controller coasts; finite transverse burn
            // changes touchdown and spends fuel.
            var s=new D.State(0,X*320000.5,new D.V(-.5,0,0),1000,Y);
            var set=Settings(cap:.5,cutoff:.3,terminal:Terminal(0));
            var a=P.Evaluate(Env(),Engines(),Limits(),s,set,
                new P.Profile(.3,0,new D.Command(0,Y)),10000);
            var b=P.Evaluate(Env(),Engines(),Limits(),s,set,
                new P.Profile(.3,.2,new D.Command(.025,Y)),10000);
            Assert.Equal(D.Outcome.ContactWithinLimits,a.Outcome);
            Assert.Equal(D.Outcome.ContactWithinLimits,b.Outcome);
            Assert.True(b.Miss>a.Miss);
            Assert.True(b.End.Mass<a.End.Mass);
        }
    }
}
