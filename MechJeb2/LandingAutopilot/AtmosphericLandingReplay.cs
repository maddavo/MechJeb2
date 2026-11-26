using System;
using System.Collections.Generic;
using System.Threading;
using D=MuMech.Landing.AtmosphericLandingDynamics;

namespace MuMech.Landing
{
    // 2.16.0-landing.20261126.duna-dynamics-m1.3
    // Non-actuating replay of an explicitly supplied command history.
    // No reset to observations, no inferred commands between intervals,
    // no acceptance of low-speed capture as a touchdown.
    internal static class AtmosphericLandingReplay
    {
        internal readonly struct Segment
        {
            internal readonly double StartUT,EndUT;
            internal readonly D.Command Command;
            internal Segment(double start,double end,D.Command command)
            {StartUT=start;EndUT=end;Command=command;}
        }

        // Full observed state, including actual normalized engine response
        // (NOT pilot/commanded throttle). The caller must establish its meaning
        // and alignment before computing force-model residuals at that state.
        internal readonly struct Observation
        {
            internal readonly D.State State;
            internal readonly bool HasMeasuredForces;
            internal readonly D.V Thrust,Drag,Lift;
            internal readonly double Flow;
            internal Observation(D.State state)
            {State=state;HasMeasuredForces=false;Thrust=Drag=Lift=default;Flow=double.NaN;}
            internal Observation(D.State state,D.V thrust,D.V drag,D.V lift,double flow)
            {State=state;HasMeasuredForces=true;Thrust=thrust;Drag=drag;Lift=lift;Flow=flow;}
            internal bool Valid=>State.Valid&&(!HasMeasuredForces||
                Thrust.Valid&&Drag.Valid&&Lift.Valid&&D.Finite(Flow)&&Flow>=0);
        }
        internal readonly struct Comparison
        {
            internal readonly double UT,PositionError,VelocityError,MassError,PointingError;
            internal readonly D.ForceSample PredictedForces,ForcesAtObservedState;
            internal readonly bool ForceResidualKnown;
            internal readonly D.V ThrustResidual,DragResidual,LiftResidual;
            internal readonly double FlowResidual;
            internal Comparison(D.Environment env,D.Propulsion propulsion,D.State predicted,Observation observed)
            {
                UT=predicted.UT;
                D.State actual=observed.State;
                PositionError=(predicted.R-actual.R).Length;
                VelocityError=(predicted.Velocity-actual.Velocity).Length;
                MassError=predicted.Mass-actual.Mass;
                PointingError=Math.Acos(Math.Max(-1,Math.Min(1,D.V.Dot(predicted.Forward,actual.Forward))));
                PredictedForces=D.Forces(env,propulsion,predicted);
                ForcesAtObservedState=D.Forces(env,propulsion,actual);
                ForceResidualKnown=observed.HasMeasuredForces&&ForcesAtObservedState.Known;
                ThrustResidual=ForceResidualKnown?ForcesAtObservedState.Thrust-observed.Thrust:default;
                DragResidual=ForceResidualKnown?ForcesAtObservedState.Drag-observed.Drag:default;
                LiftResidual=ForceResidualKnown?ForcesAtObservedState.Lift-observed.Lift:default;
                FlowResidual=ForceResidualKnown?ForcesAtObservedState.MassFlow-observed.Flow:double.NaN;
            }
        }
        internal sealed class Report
        {
            internal readonly D.Outcome Outcome;
            internal readonly D.State End;
            internal readonly double MinimumClearance;
            internal readonly int Steps,CompletedSegments;
            internal readonly bool HistoryCompleted;
            internal readonly IReadOnlyList<Comparison> Comparisons;
            internal Report(D.Outcome outcome,D.State end,double clearance,int steps,int segments,
                bool completed,List<Comparison> comparisons)
            {
                Outcome=outcome;End=end;MinimumClearance=clearance;Steps=steps;CompletedSegments=segments;
                HistoryCompleted=completed;Comparisons=Array.AsReadOnly(comparisons.ToArray());
            }
        }

        // Segment boundaries and observation UTs must share the same verified
        // epoch. Exact adjacency is intentional; gaps are not implicit coasts.
        // Observations may occur inside segments; subdivide steps at those UTs.
        // A complete command history is not a safe landing unless the model
        // reports contact within limits (and real-vessel parity is separate).
        internal static Report Run(D.Environment env,D.Propulsion propulsion,D.Limits limits,
            D.State initial,Segment[] history,Observation[] observations,
            double safeVertical=1,double safeHorizontal=1,double safeAngle=5*Math.PI/180,
            double safeRate=Math.PI/180,int maxSteps=100000,CancellationToken cancellation=default)
        {
            var comparisons=new List<Comparison>();
            D.State state=initial;
            double minimum=env==null?double.NaN:env.PhysicalClearance(initial);
            int steps=0,segments=0,index=0;
            Report Finish(D.Outcome outcome,bool completed=false)=>
                new Report(outcome,state,minimum,steps,segments,completed,comparisons);
            if(env==null||propulsion==null||limits==null||!env.Valid||!limits.Valid||!initial.Valid||
                initial.Mass<limits.ReserveMass||history==null||history.Length==0||observations==null||
                maxSteps<1||maxSteps>1000000||!D.Finite(safeVertical)||safeVertical<0||
                !D.Finite(safeHorizontal)||safeHorizontal<0||!D.Finite(safeAngle)||safeAngle<0||
                !D.Finite(safeRate)||safeRate<0)return Finish(D.Outcome.InvalidInput);
            history=(Segment[])history.Clone();observations=(Observation[])observations.Clone();
            double cursor=initial.UT;
            foreach(Segment segment in history)
            {
                if(!D.Finite(segment.StartUT)||!D.Finite(segment.EndUT)||segment.StartUT!=cursor||
                    segment.EndUT<=segment.StartUT||!segment.Command.Valid)
                    return Finish(D.Outcome.InvalidInput);
                cursor=segment.EndUT;
            }
            double last=double.NegativeInfinity;
            foreach(Observation observation in observations)
            {
                double t=observation.State.UT;
                if(!observation.Valid||t<initial.UT||t>cursor||t<=last)
                    return Finish(D.Outcome.InvalidInput);
                last=t;
            }
            if(cancellation.IsCancellationRequested)return Finish(D.Outcome.Cancelled);
            void Observe()
            {
                while(index<observations.Length&&observations[index].State.UT==state.UT)
                    comparisons.Add(new Comparison(env,propulsion,state,observations[index++]));
            }
            Observe();
            foreach(Segment segment in history)
            {
                while(state.UT<segment.EndUT)
                {
                    if(cancellation.IsCancellationRequested)return Finish(D.Outcome.Cancelled);
                    if(steps>=maxSteps)return Finish(D.Outcome.Horizon);
                    double bound=segment.EndUT;
                    if(index<observations.Length)bound=Math.Min(bound,observations[index].State.UT);
                    double dt=Math.Min(limits.Step,bound-state.UT);
                    if(dt<=0||state.UT+dt<=state.UT)return Finish(D.Outcome.InvalidInput);
                    D.Result result=D.Advance(env,propulsion,limits,state,segment.Command,dt,
                        safeVertical,safeHorizontal,safeAngle,safeRate);
                    steps++;state=result.End;
                    minimum=Math.Min(minimum,result.MinimumClearance);
                    Observe();
                    if(result.Status!=D.Outcome.Advanced)
                        return Finish(result.Status);
                }
                if(state.UT!=segment.EndUT)return Finish(D.Outcome.InvalidInput);
                segments++;
            }
            return Finish(D.Outcome.Advanced,true);
        }
    }
}
