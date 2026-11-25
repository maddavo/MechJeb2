using System;
using Xunit;
using MuMech.Landing;
using D=MuMech.Landing.AtmosphericLandingDynamics;

namespace MechJebLibTest.LandingPredictionTests
{
    public class DunaDynamicScreenTests
    {
        private static D.V X=>new D.V(1,0,0);
        private static DunaBrakingHandoff.Result R(bool safe,string reason)
            =>new DunaBrakingHandoff.Result(safe,reason,500,500,10,20);
        [Fact]
        public void BothTerminalFailuresAreNotADeadline()
        {
            Assert.Equal(DunaDynamicScreen.Decision.TerminalUnresolved,DunaDynamicScreen.Classify(
                R(false,"terminal_authority_unresolved"),R(false,"terminal_authority_unresolved")));
            Assert.Equal(DunaDynamicScreen.Decision.HoldUnresolved,DunaDynamicScreen.Classify(
                R(false,"UnknownCapability"),R(false,"UnknownCapability")));
            Assert.Equal(DunaDynamicScreen.Decision.Emergency,DunaDynamicScreen.Classify(
                R(false,"ContactUnsafe"),R(false,"terminal_authority_unresolved")));
            Assert.Equal(DunaDynamicScreen.Decision.BrakeDeadline,DunaDynamicScreen.Classify(
                R(false,"ContactUnsafe"),R(true,"dynamic_capture_envelope")));
            Assert.Equal(DunaDynamicScreen.Decision.Emergency,DunaDynamicScreen.Classify(
                R(false,"ContactUnsafe"),R(false,"ContactUnsafe")));
            Assert.Equal(DunaDynamicScreen.Decision.Continue,DunaDynamicScreen.Classify(
                R(true,"dynamic_capture_envelope"),default));
        }

        // Actual trace-derived vectors and mass at QS226 commitment.
        // Atmosphere curve, accessible fuel and effective flow here are
        // explicitly illustrative. This is not a reconstructed flight.
        [Fact]
        public void RecordedCommitmentCanPropagateDespiteStaticHoverFailure()
        {
            const double radius=320000,mu=301363211975.0978,terrain=330511.59624280804;
            const double mass=245432.22907651216,accel=3.1887950554275446;
            D.V up=new D.V(.07787778011820973,.16409058304515,.9833663264113544);
            D.V pos=up.Unit*413750.0265463413;
            D.V vel=new D.V(-552.6945351034458,539.6030334895553,-107.4279572502721);
            D.V omega=new D.V(0,-9.590034483906332e-5,0);
            var old=DunaBrakingHandoff.Probe(new DunaBrakingHandoff.Vec(pos.X,pos.Y,pos.Z),
                new DunaBrakingHandoff.Vec(vel.X,vel.Y,vel.Z),new DunaBrakingHandoff.Vec(omega.X,omega.Y,omega.Z),
                mu,terrain,accel,30.194323782889);
            Assert.Equal("insufficient_hover_authority",old.Reason);
            Assert.Equal(0,old.Elapsed);
            double f=mass*accel*DunaDynamicScreen.Margin;
            var force=new D.Curve(new D.Knot(0,f),new D.Knot(7000,f*.98));
            var pressure=new D.Curve(new D.Knot(0,7000),new D.Knot(50000,0));
            var density=new D.Curve(new D.Knot(0,0),new D.Knot(50000,0));
            var env=new D.Environment(radius,mu,omega,50000,pressure,density,0,terrain,0);
            var prop=D.Propulsion.FromAllocatedEnvelope(force,f/(400*9.80665));
            var limits=new D.Limits(145000,3*Math.PI/180,0,0,1);
            var state=new D.State(0,pos,vel,mass,new D.V(-.2022141069173813,.9557151198387146,.2138177752494812).Unit);
            var delayed=DunaDynamicScreen.Probe(env,prop,limits,state,55,terminalRadius:radius);
            Assert.True(delayed.Safe,delayed.Reason);
            Assert.True(delayed.Elapsed>30);
            Assert.True(delayed.MinimumClearance>300);
            Assert.Equal(DunaDynamicScreen.Decision.Continue,DunaDynamicScreen.Classify(delayed,default));
        }

        [Fact]
        public void FiniteCorrectionMustAimBeforeDeliveryAndFitBothWindows()
        {
            var env=new D.Environment(320000,0,default,0,null,null,0,320000,0);
            var prop=D.Propulsion.FromAllocatedEnvelope(new D.Curve(new D.Knot(0,10000)),1);
            var lim=new D.Limits(500,.1,0,0,.1);
            var state=new D.State(0,X*330000,new D.V(-50,0,0),1000,X);
            var correction=new D.Command(.5,new D.V(0,1,0));
            var result=D.Screen(env,prop,lim,state,2,correction,5,100,correctionLead:1);
            Assert.Equal(D.Outcome.UnknownCapability,result.Status);
            Assert.False(DunaDynamicScreen.CorrectionFits(0,4,2,5,1,3,10));
            Assert.False(DunaDynamicScreen.CorrectionFits(0,4,2,20,1,3,9));
            Assert.True(DunaDynamicScreen.CorrectionFits(0,4,2,20,1,3,10));
        }

        [Fact]
        public void NoAltitudeOnlyTerminalTransfer()
        {
            Assert.False(DunaDynamicScreen.CanTerminalHandoff(150,10,3,100));
            Assert.False(DunaDynamicScreen.CanTerminalHandoff(10,2.8,3,100));
            Assert.False(DunaDynamicScreen.CanTerminalHandoff(10,10,3,-1));
            Assert.True(DunaDynamicScreen.CanTerminalHandoff(10,4,3,100));
            Assert.False(DunaDynamicScreen.CanTerminalHandoff(10,4,3,84600));
            Assert.True(DunaDynamicScreen.CanCaptureRecovery(10,4,3,84600));
        }
        [Fact]
        public void RecordedNovember26UnknownPressureCannotCommitBraking()
        {
            // Reported outcomes at UT 31155016.770807, not a complete flight reconstruction.
            var delayed=new DunaBrakingHandoff.Result(false,"terminal_pressure_unknown",74901.91,74901.91,14.13,199.865);
            var immediate=new DunaBrakingHandoff.Result(true,"dynamic_capture_envelope",76512.64,double.NaN,double.NaN,double.NaN);
            Assert.Equal(DunaDynamicScreen.ProbeStatus.TerminalUnresolved,DunaDynamicScreen.Status(delayed));
            Assert.Equal(DunaDynamicScreen.Decision.TerminalUnresolved,DunaDynamicScreen.Classify(delayed,immediate));
            Assert.False(DunaDynamicScreen.UnresolvedEmergency(31155016.770807,31155477.858,42.865,96058.9,true));
            Assert.False(DunaDynamicScreen.UnresolvedEmergency(31155018.770807,31155477.858,42.865,96000,true));
            Assert.True(DunaBrakingHandoff.EntryFits(31155016.770807,42.865,31155430.020));
        }

        [Theory]
        [InlineData("UnknownCapability")]
        [InlineData("Horizon")]
        [InlineData("InvalidInput")]
        [InlineData("ContactWithinLimits")]
        [InlineData("NoControllableThrust")]
        [InlineData("live_model_unsupported_engine")]
        [InlineData("terminal_authority_unresolved")]
        [InlineData("terminal_fuel_reserve")]
        [InlineData("terminal_authority_unknown")]
        public void MissingOrUnresolvedDelayedResultNeverEstablishesDeadline(string reason)
        {
            var decision=DunaDynamicScreen.Classify(R(false,reason),R(true,"dynamic_capture_envelope"));
            Assert.NotEqual(DunaDynamicScreen.Decision.BrakeDeadline,decision);
            Assert.NotEqual(DunaDynamicScreen.Decision.Emergency,decision);
            Assert.NotEqual(DunaDynamicScreen.Decision.Continue,decision);
        }

        [Fact]
        public void TerminalPressureLookupIsIndependentOfCaptureDirection()
        {
            const double radius=320000,lower=310123.456;
            var p=new D.Curve(new D.Knot(lower-radius,8000),new D.Knot(50000,0));
            var rho=new D.Curve(new D.Knot(lower-radius,0),new D.Knot(50000,0));
            var env=new D.Environment(radius,301363211975.09772,default,50000,p,rho,0,330511,0);
            var prop=D.Propulsion.FromAllocatedEnvelope(new D.Curve(new D.Knot(0,20000),new D.Knot(8000,19000)),1);
            var lim=new D.Limits(500,1,0,0,1);
            bool foundRoundoff=false;
            for(int k=1;k<=128;k++)
            {
                var dir=new D.V(k*.021+1,Math.Sin(k*.37)+2,Math.Cos(k*.81)+2).Unit;
                var pos=dir*413750;
                if(!env.Air(pos.Unit*lower,out _,out _))foundRoundoff=true;
                var state=new D.State(0,pos,dir*(-5),1000,dir);
                var result=DunaDynamicScreen.Probe(env,prop,lim,state,0,terminalRadius:lower);
                Assert.True(result.Safe,result.Reason);
            }
            Assert.True(foundRoundoff,"Fixture must exercise old vector round-trip boundary failure");
            Assert.True(env.AirAtAltitude(lower-radius,out double pressure,out _));
            Assert.Equal(8000.0,pressure);
            Assert.False(env.AirAtAltitude(lower-radius-.001,out _,out _));
        }

        [Fact]
        public void UnknownFallbackDependsOnGeometricUrgencyNotElapsedRetryTime()
        {
            Assert.False(DunaDynamicScreen.UnresolvedEmergency(100,500,45,97000,true));
            Assert.False(DunaDynamicScreen.UnresolvedEmergency(300,500,45,50000,true));
            Assert.True(DunaDynamicScreen.UnresolvedEmergency(460,500,45,12000,true));
            Assert.False(DunaDynamicScreen.UnresolvedEmergency(460,500,45,12000,false));
            Assert.True(DunaDynamicScreen.UnresolvedEmergency(100,double.NaN,45,900,true));
        }
    }
}
