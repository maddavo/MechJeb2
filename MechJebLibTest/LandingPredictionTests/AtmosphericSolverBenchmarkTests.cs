using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using Xunit;
using Xunit.Abstractions;
using D=MuMech.Landing.AtmosphericLandingDynamics;
using F=MuMech.Landing.AtmosphericFiniteTimePlanner;
using P=MuMech.Landing.AtmosphericFeasibleRangePlanner;
using T=MuMech.Landing.AtmosphericTerminalReplay;
using Convex=MuMech.Landing.AtmosphericConvexPlanner;
using Benchmark=MuMech.Landing.AtmosphericSolverBenchmark;
using Replay=MuMech.Landing.AtmosphericLandingReplay;
using Uncertainty=MuMech.Landing.AtmosphericUncertaintyReplay;

namespace MechJebLibTest.LandingPredictionTests
{
    public class AtmosphericSolverBenchmarkTests
    {
        private readonly ITestOutputHelper output;
        public AtmosphericSolverBenchmarkTests(ITestOutputHelper output){this.output=output;}
        private static D.V X=>new D.V(1,0,0);
        private static D.V Y=>new D.V(0,1,0);
        private static D.Environment Env(double mu=0,double drag=0)=>
            drag==0?new D.Environment(320000,mu,default,0,null,null,0,320000,0):
            new D.Environment(320000,mu,default,50000,
                new D.Curve(new D.Knot(0,20000),new D.Knot(50000,0)),
                new D.Curve(new D.Knot(0,.15),new D.Knot(50000,0)),drag,320000,0);
        private static D.Propulsion Engine(double flow=1)=>new D.Propulsion(true,
            new D.Engine(flow,1,9.80665,default,new D.Curve(new D.Knot(0,390),new D.Knot(1,390))));
        private static D.State State(double h=.53,double down=.5,double lateral=0,double mass=1000)=>
            new D.State(0,X*(320000+h),X*(-down)+Y*lateral,mass,X);
        private static F.Settings Settings(D.Environment env,D.Propulsion engine,double cap=1,double min=.5,
            double max=1,double coast=.4,double horizon=4,D.V? target=null,int candidates=24,int steps=45000)
        {
            double g=env.Mu/(env.Radius*env.Radius);
            var term=new T.Settings(.2,.5,g/9.81,0,st=>{
                if(!env.Air(st.UT,st.R,out double pressure,out _))return default;
                var a=engine.At(pressure);double raw=a.Force/st.Mass;
                return new T.ControllerAuthority(raw,raw*cap,cap,false);
            },(st,dir)=>0,(ut,pos)=>320000);
            var common=new P.Settings(target??X,horizon,coast,100,term,
                new T.Memory(T.Phase.FinalDescent,T.Mode.Off,0,0,0,0),
                intervals:4,refine:1,maxCandidates:candidates,maxSteps:steps,captureClearance:0,
                commandCap:cap);
            return new F.Settings(common,320000,approachHeight:15,approachSpeed:.5,minPowered:min,maxPowered:max,
                controlTick:.5,coastIntervals:3,durationIntervals:4,refinements:1);
        }

        [Fact]
        public void ConvexMinimumEnergyConstantAccelerationMeetsBothEndpointRows()
        {
            var r=Convex.Solve(X*25,X*5,default,10,2,X,Math.PI,12,160,1e-8);
            Assert.True(r.Converged);
            Assert.InRange(r.PositionResidual,0,1e-6);
            Assert.InRange(r.VelocityResidual,0,1e-6);
            Assert.All(r.Controls,u=>{
                Assert.Equal(.25,u.X,7);
                Assert.InRange(Math.Abs(u.Y)+Math.Abs(u.Z),0,1e-10);
            });
        }

        [Fact]
        public void ConvexProjectionRespectsPointingAndThrustBall()
        {
            var v=Convex.ProjectControl(Y*5,X,Math.PI/6);
            Assert.Equal(1,v.Length,9);
            Assert.Equal(Math.Cos(Math.PI/6),v.X,9);
            Assert.Equal(Math.Sin(Math.PI/6),v.Y,9);
            Assert.Equal(0,Convex.ProjectControl(X*(-1),X,Math.PI/6).Length);
            Assert.Equal(1,Convex.ProjectControl(Y*5,X,Math.PI).Length,9);
            Assert.False(Convex.ProjectControl(Y,X,2).Valid);
        }

        [Fact]
        public void ConvexConstrainedSolutionAndInfeasibleResidualAreExplicit()
        {
            // Constant lateral acceleration is impossible in a zero-angle
            // cone. A bounded residual must never become a valid command.
            var bad=Convex.Solve(Y*.25,Y*.5,default,1,1,X,0,8,30);
            Assert.False(bad.Converged);
            Assert.True(bad.PositionResidual>0&&bad.VelocityResidual>0);
            var good=Convex.Solve(X*.25,X*.5,default,1,1,X,0,8,30);
            Assert.True(good.Converged);
            Assert.All(good.Controls,u=>Assert.InRange(u.Length,0,1));
            Assert.False(Convex.Solve(X*100,X*100,default,1,1,X,Math.PI).Converged);
            Assert.False(Convex.Solve(X*.25,X*.5,default,1,1,X,Math.PI,
                token:new CancellationToken(true)).Converged);
        }

        [Fact]
        public void ConvexCommandUsesActualPressureMassAndAttitudeGate()
        {
            var env=Env();var engine=Engine();var s=State(1000,10);
            var set=Settings(env,engine);
            double t=10;
            var target=s.R+s.Velocity*t+X*25;
            var terminal=s.Velocity+X*5;
            var c=Convex.Command(env,engine,s,set,target,terminal,t,out double requested,out _,default);
            Assert.True(c.Valid);
            Assert.Equal(.5,requested,7);
            Assert.Equal(.5*s.Mass/engine.At(0).Force,c.Throttle,7);
            var turned=new D.State(s.UT,s.R,s.Velocity,s.Mass,Y);
            var gated=Convex.Command(env,engine,turned,set,target,terminal,t,out _,out _,default);
            Assert.True(gated.Valid);
            Assert.Equal(0,gated.Throttle);
        }

        [Fact]
        public void ConvexSolverActuallyExecutesPoweredNonlinearCandidate()
        {
            var env=Env();var engine=Engine();var initial=State(1000,20);
            var set=Settings(env,engine,min:90,max:90,coast:0,horizon:150);
            // There is a real first powered interval, whether or not the
            // subsequent candidate reaches contact. Never count a direct
            // terminal-only fixture as exercising the convex powered law.
            var c=F.Evaluate(env,engine,new D.Limits(100,20,0,0,.5),initial,set,
                new F.Profile(0,90),1,true,default,true);
            Assert.Single(c.Reference);
            Assert.True(c.Reference[0].Command.Throttle>0);
            Assert.Equal(D.Outcome.Horizon,c.Result.Outcome);
            Assert.False(c.Result.Feasible);
        }

        private static string N(double n)=>D.Finite(n)?n.ToString("G12",CultureInfo.InvariantCulture):"NA";

        private static string ContactState(D.Environment env,D.State state)
        {
            D.V up=state.R.Unit,w=env.Surface(state.R,state.Velocity);
            double vertical=D.V.Dot(w,up),horizontal=(w-up*vertical).Length;
            double angle=Math.Acos(Math.Max(-1,Math.Min(1,D.V.Dot(state.Forward,up))))*180/Math.PI;
            return "ut="+N(state.UT)+" clearance="+N(env.PhysicalClearance(state))+
                " vertical="+N(vertical)+" horizontal="+N(horizontal)+" angle_deg="+N(angle)+
                " rate_deg_s="+N(state.PointingRate*180/Math.PI)+" mass="+N(state.Mass)+
                " attained_throttle="+N(state.Throttle);
        }

        private static Replay.Report ReplayAt(D.Environment env,D.Propulsion engine,D.Limits lim,D.State state,
            F.Settings settings,F.Candidate c,double step)
        {
            var common=settings.Common;
            var finer=new D.Limits(lim.ReserveMass,lim.TurnRate,lim.EngineRise,lim.EngineFall,step);
            return Replay.Run(env,engine,finer,state,c.Commands.ToArray(),Array.Empty<Replay.Observation>(),
                common.SafeVertical,common.SafeHorizontal,common.SafeAngle,common.SafeRate,80000);
        }

        // Diagnostic only: do not change either production law or the
        // existing benchmark acceptance criteria to make a fixture pass.
        [Fact]
        public void DiagnoseM52ReplayFailuresAndAlternateProfiles()
        {
            var rows=new List<string>();
            var duna=Env(301363211975.0978);var heavy=Engine(350);
            var thin=Env(301363211975.0978,4);var e=Engine();
            var fixtures=new[]{
                (name:"vertical_6km",env:duna,engine:heavy,limits:new D.Limits(184000,Math.PI/30,.1,.1,.5),
                    state:State(6080,15,0,220000),settings:Settings(duna,heavy,min:90,max:130,coast:10,horizon:400)),
                (name:"thin_atmosphere_crossrange",env:thin,engine:e,limits:new D.Limits(700,Math.PI/15,.1,.1,.5),
                    state:State(500,15,20),settings:Settings(thin,e,min:20,max:90,coast:5,horizon:200,target:(X+Y*.0001).Unit))
            };
            foreach(var f in fixtures)
            foreach(bool convex in new[]{false,true})
            {
                string label=f.name+" "+(convex?"C":"B");
                var set=f.settings;var common=set.Common;
                var search=F.Search(f.env,f.engine,f.limits,f.state,set,convex:convex);
                if(search.Best==null){rows.Add(label+" no nominal candidate: "+search.Status);continue;}
                var selected=search.Best.Profile;
                var coarse=F.Evaluate(f.env,f.engine,f.limits,f.state,set,selected,80000,true,default,convex);
                if(!coarse.Result.Feasible){rows.Add(label+" reconstruction failed: "+coarse.Result.Outcome);continue;}
                rows.Add(label+" selected coast="+N(selected.Coast)+" duration="+N(selected.Duration)+
                    " terminal_start="+N(coarse.Result.TerminalStartUT)+" limits_v="+N(common.SafeVertical)+
                    " h="+N(common.SafeHorizontal)+" angle_deg="+N(common.SafeAngle*180/Math.PI)+
                    " rate_deg_s="+N(common.SafeRate*180/Math.PI));
                rows.Add(label+" nominal "+ContactState(f.env,coarse.Result.End));
                foreach(double dt in new[]{.5,.25,.125,.0625})
                {
                    var replay=ReplayAt(f.env,f.engine,f.limits,f.state,set,coarse,dt);
                    rows.Add(label+" fixed_history dt="+N(dt)+" outcome="+replay.Outcome+
                        " "+ContactState(f.env,replay.End)+" completed="+replay.CompletedSegments+
                        " pos_delta="+N((replay.End.R-coarse.Result.End.R).Length)+
                        " vel_delta="+N((replay.End.Velocity-coarse.Result.End.Velocity).Length));
                    if(dt==.25&&replay.Outcome==D.Outcome.UnknownCapability&&replay.CompletedSegments<coarse.Commands.Count)
                    {
                        var seg=coarse.Commands[replay.CompletedSegments];var st=replay.End;
                        double failedDt=Math.Min(dt,seg.EndUT-st.UT);
                        double midAsl=(st.R+st.Velocity*(failedDt*.5)).Length-f.env.Radius;
                        bool midAir=f.env.AirAtAltitude(midAsl,out double p,out double rho);
                        rows.Add(label+" failed_step dt="+N(failedDt)+" segment_end="+N(seg.EndUT)+
                            " midpoint_asl="+N(midAsl)+" midpoint_air_known="+midAir+
                            " pressure="+N(p)+" density="+N(rho)+" current_forces_known="+D.Forces(f.env,f.engine,st).Known);
                        var tiny=new D.Limits(f.limits.ReserveMass,f.limits.TurnRate,f.limits.EngineRise,f.limits.EngineFall,dt);
                        for(int split=1;split<=4;split++)
                        {
                            var result=D.Advance(f.env,f.engine,tiny,st,seg.Command,failedDt/Math.Pow(2,split),
                                common.SafeVertical,common.SafeHorizontal,common.SafeAngle,common.SafeRate);
                            rows.Add(label+" failed_step_subdivision "+split+" outcome="+result.Status+" "+ContactState(f.env,result.End));
                        }
                    }
                }
                foreach(double dt in new[]{.25,.125,.0625})
                {
                    var fineLim=new D.Limits(f.limits.ReserveMass,f.limits.TurnRate,f.limits.EngineRise,f.limits.EngineFall,dt);
                    var rebuilt=F.Evaluate(f.env,f.engine,fineLim,f.state,set,selected,80000,true,default,convex);
                    rows.Add(label+" rebuild dt="+N(dt)+" outcome="+rebuilt.Result.Outcome+
                        " "+ContactState(f.env,rebuilt.Result.End));
                    if(rebuilt.Result.Feasible)
                    {
                        var rep=ReplayAt(f.env,f.engine,fineLim,f.state,set,rebuilt,dt*.5);
                        rows.Add(label+" rebuilt_replay dt="+N(dt*.5)+" outcome="+rep.Outcome+" "+ContactState(f.env,rep.End));
                    }
                }
                int tested=0,nominal=0,finerContact=0,finePassed=0,allPassed=0;
                // Original grid plus the selected refined profile. This is
                // a bounded sample, not a claim of full reachable coverage.
                var profiles=new List<F.Profile>{selected};
                for(int i=0;i<=set.CoastIntervals;i++)
                for(int j=0;j<=set.DurationIntervals;j++)
                {
                    var p=new F.Profile(common.MaxIgnitionDelay*i/set.CoastIntervals,
                        set.MinPoweredSeconds+(set.MaxPoweredSeconds-set.MinPoweredSeconds)*j/set.DurationIntervals);
                    if(!profiles.Any(x=>Math.Abs(x.Coast-p.Coast)<1e-9&&Math.Abs(x.Duration-p.Duration)<1e-9))profiles.Add(p);
                }
                foreach(var profile in profiles)
                {
                    tested++;
                    var c=F.Evaluate(f.env,f.engine,f.limits,f.state,set,profile,80000,true,default,convex);
                    if(!c.Result.Feasible)continue;
                    nominal++;
                    var replay=ReplayAt(f.env,f.engine,f.limits,f.state,set,c,.25);
                    if(replay.Outcome!=D.Outcome.ContactWithinLimits)continue;
                    finerContact++;
                    bool valid=(c.Result.End.R-replay.End.R).Length<=20&&
                        Math.Abs(c.Result.End.Mass-replay.End.Mass)<=Math.Max(.5,f.state.Mass*.0005)&&
                        Math.Abs(c.Result.End.UT-replay.End.UT)<=set.ControlTick+common.Terminal.Tick&&
                        (c.Result.Miss>common.TargetTolerance||P.Miss(f.env,f.state,replay.End,common.TargetAtInitialUT)<=common.TargetTolerance);
                    if(!valid)continue;
                    finePassed++;
                    var tracker=new MuMech.Landing.AtmosphericPlanTracker(f.env,f.engine,f.limits,set,c);
                    var screen=Uncertainty.Validate(f.env,f.engine,f.limits,tracker,80000);
                    if(screen.Passed)allPassed++;
                    rows.Add(label+" alternate coast="+N(profile.Coast)+" duration="+N(profile.Duration)+
                        " fuel="+N(f.state.Mass-c.Result.End.Mass)+" uncertainty="+screen.Passed+
                        " reason="+screen.Reason);
                }
                rows.Add(label+" sweep tested="+tested+" nominal="+nominal+" finer_contact="+finerContact+
                    " fine_passed="+finePassed+" all_passed="+allPassed);
            }
            foreach(string row in rows)output.WriteLine(row);
            string file=System.Environment.GetEnvironmentVariable("DUNA_BENCHMARK_OUTPUT");
            if(!string.IsNullOrEmpty(file))File.WriteAllLines(Path.ChangeExtension(file,".replay.tsv"),rows);
        }

        [Fact]
        public void RankedValidationFindsRobustAlternativesToRejectedNominalOptimum()
        {
            var env=Env(301363211975.0978);var engine=Engine(350);
            var limits=new D.Limits(184000,Math.PI/30,.1,.1,.5);
            var start=State(6080,15,0,220000);
            var set=Settings(env,engine,min:90,max:130,coast:10,horizon:400);
            foreach(bool convex in new[]{false,true})
            {
                var search=F.Search(env,engine,limits,start,set,convex:convex);
                Assert.NotNull(search.Best);
                Assert.True(search.Ranked.Count>1);
                Assert.True(F.Validate(env,engine,limits,start,set,search.Best,30000,20,110,
                    out var first,out _,convex:convex));
                var firstScreen=Uncertainty.Validate(env,engine,limits,
                    new MuMech.Landing.AtmosphericPlanTracker(env,engine,limits,set,first),120000);
                // This exact M5.4 fixture has robust alternatives despite
                // its rejected nominal optimum. If the optimum itself
                // becomes robust, selecting it is also correct.
                var selected=F.SelectValidated(env,engine,limits,start,set,search,30000,120000,
                    20,110,convex:convex);
                Assert.NotNull(selected.Candidate);
                Assert.True(selected.Scenarios.Passed);
                Assert.True(selected.FineReplayPassed);
                Assert.InRange(selected.ReplaySteps,1,30000);
                Assert.InRange(selected.ScenarioSteps,1,120000);
                Assert.InRange(selected.Attempts,firstScreen.Passed?1:2,12);
                Assert.True(selected.Candidate.Result.End.Mass>=limits.ReserveMass);
                var cancelled=F.SelectValidated(env,engine,limits,start,set,search,30000,120000,
                    20,110,new CancellationToken(true),convex);
                Assert.Null(cancelled.Candidate);
                Assert.Equal(0,cancelled.Attempts);
            }
        }

        [Fact]
        public void CommonBenchmarkReportsActualOutcomesWithoutSelectingProductionSolver()
        {
            var rows=new List<string>{"fixture\tsolver\tstatus\tnominal\tfine\tscenarios\tvalidated\tattempts\tsearch_steps\tvalidation_steps\tsearch_ms\ttotal_ms\tmiss_m\tfuel_kg\tfirst_powered_ut\tmax_sampled_miss_m\tmin_sampled_contact_mass_kg\treason"};
            var fixtures=new List<(string name,D.Environment env,D.Propulsion engine,D.Limits limits,D.State state,F.Settings settings)>();
            var env=Env();var e=Engine();
            fixtures.Add(("near_contact",env,e,new D.Limits(100,20,0,0,.1),State(),Settings(env,e)));
            var duna=Env(301363211975.0978);var heavy=Engine(350);
            fixtures.Add(("vertical_6km",duna,heavy,new D.Limits(184000,Math.PI/30,.1,.1,.5),
                State(6080,15,0,220000),Settings(duna,heavy,min:90,max:130,coast:10,horizon:400)));
            var thin=Env(301363211975.0978,4);
            fixtures.Add(("thin_atmosphere_crossrange",thin,e,new D.Limits(700,Math.PI/15,.1,.1,.5),
                State(500,15,20),Settings(thin,e,min:20,max:90,coast:5,horizon:200,target:(X+Y*.0001).Unit)));
            fixtures.Add(("limited_fuel",duna,heavy,new D.Limits(219999,Math.PI/30,.1,.1,.5),
                State(6080,15,0,220000),Settings(duna,heavy,min:90,max:130,coast:10,horizon:400)));
            fixtures.Add(("unreachable_target",env,e,new D.Limits(100,20,0,0,.1),
                State(),Settings(env,e,target:Y)));
            foreach(var f in fixtures)
            {
                var trials=Benchmark.Compare(f.env,f.engine,f.limits,f.state,f.settings,80000,10000);
                Assert.Equal(3,trials.Length);
                foreach(var t in trials)
                {
                    Assert.InRange(t.SearchSteps,0,f.settings.Common.MaxSteps);
                    Assert.InRange(t.ValidationSteps,0,80000);
                    if(t.Validated)
                    {
                        Assert.True(t.Nominal.Value.Feasible);
                        Assert.True(t.Nominal.Value.End.Mass>=f.limits.ReserveMass);
                        Assert.True(t.SampledMinimumMass>=f.limits.ReserveMass);
                    }
                    if(f.name=="limited_fuel")Assert.False(t.Validated);
                    if((f.name=="near_contact"||f.name=="vertical_6km"||f.name=="thin_atmosphere_crossrange")&&
                        t.Solver==Benchmark.Method.FiniteTimeShooting)Assert.True(t.Validated,t.Reason);
                    var a=t.Nominal;
                    rows.Add(string.Join("\t",f.name,t.Solver,t.Status,a.HasValue,t.FineReplayPassed,t.ScenarioPassed,t.Validated,
                        t.Attempts,t.SearchSteps,t.ValidationSteps,N(t.SearchMs),N(t.TotalMs),
                        N(a.HasValue?a.Value.Miss:double.NaN),
                        N(a.HasValue?f.state.Mass-a.Value.End.Mass:double.NaN),
                        N(a.HasValue?a.Value.FirstPoweredIntervalUT:double.NaN),
                        N(t.SampledWorstMiss),N(t.SampledMinimumMass),t.Reason.Replace('\t',' ').Replace('\n',' ')));
                }
            }
            Assert.Equal(16,rows.Count);
            foreach(string row in rows)output.WriteLine(row);
            string file=System.Environment.GetEnvironmentVariable("DUNA_BENCHMARK_OUTPUT");
            if(!string.IsNullOrEmpty(file))File.WriteAllLines(file,rows);
        }
    }
}
