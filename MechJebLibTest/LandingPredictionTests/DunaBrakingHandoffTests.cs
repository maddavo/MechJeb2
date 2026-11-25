using System;
using MuMech.Landing;
using Xunit;

namespace MechJebLibTest.LandingPredictionTests
{
    public class DunaBrakingHandoffTests
    {
        private const double Radius=320000, Mu=2.94*Radius*Radius;
        private static DunaBrakingHandoff.Result Probe(double height,double vertical,double horizontal,double delay,
            double acceleration=12,double terrain=Radius)
        {
            return DunaBrakingHandoff.Probe(new DunaBrakingHandoff.Vec(Radius+height,0,0),
                new DunaBrakingHandoff.Vec(vertical,horizontal,0),new DunaBrakingHandoff.Vec(0,0,0),
                Mu,terrain,acceleration,delay);
        }

        [Fact]
        public void EarlierEffectiveThrustCanPreserveClearance()
        {
            var immediate=Probe(8000,-200,0,0);
            var delayed=Probe(8000,-200,0,30);
            Assert.True(immediate.Safe);
            Assert.False(delayed.Safe);
            Assert.Equal("terminal_clearance_exhausted",delayed.Reason);
            Assert.True(immediate.EndClearance>=DunaBrakingHandoff.TerminalReserve);
            Assert.True(immediate.EndSpeed<=DunaBrakingHandoff.CaptureSpeed);
        }

        [Fact]
        public void VectorGeometryNotTotalSpeedAloneDeterminesResult()
        {
            var radial=Probe(3000,-200,0,0);
            var horizontal=Probe(3000,0,200,0);
            Assert.False(radial.Safe);
            Assert.True(horizontal.Safe);
        }

        [Fact]
        public void HigherTerrainConsumesReserve()
        {
            Assert.True(Probe(8000,-200,0,0).Safe);
            Assert.False(Probe(8000,-200,0,0,12,Radius+6000).Safe);
        }

        [Fact]
        public void InvalidAndInsufficientAuthorityCannotAuthorizeCorrection()
        {
            Assert.Equal("invalid_input",Probe(8000,-200,0,double.NaN).Reason);
            Assert.Equal("invalid_input",Probe(8000,-200,0,0,double.NaN).Reason);
            Assert.Equal("insufficient_hover_authority",Probe(8000,-200,0,0,3).Reason);
            Assert.False(Probe(8000,-200,0,1200).Safe);
        }

        [Fact]
        public void MeasuredPreparationAndEngineResponseConsumeSlack()
        {
            double fallback=DunaBrakingHandoff.TurnReserve(180,double.NaN);
            double measured=DunaBrakingHandoff.TurnReserve(180,30);
            Assert.Equal(30,fallback);
            Assert.Equal(45,measured);
            Assert.True(DunaBrakingHandoff.PreparationReserve(measured,2,1,1,.02)>measured+4);
            Assert.True(double.IsNaN(DunaBrakingHandoff.PreparationReserve(measured,double.NaN,0,0,.02)));
        }

        [Fact]
        public void CommitmentCannotBeReplacedByASecondDecision()
        {
            var c=new DunaBrakingCommitment();
            Assert.False(c.Committed);
            c.Commit(10,"deadline",true);
            c.Commit(20,"converged",false);
            Assert.True(c.Committed);
            Assert.Equal(10,c.CommitUT);
            Assert.Equal("deadline",c.Reason);
            Assert.True(c.RequiresThrust(false,true,100));
            Assert.False(c.RequiresThrust(true,true,100));
            Assert.True(c.RequiresThrust(true,false,100));
            Assert.False(c.RequiresThrust(true,false,10));
            Assert.False(c.RequiresThrust(false,false,double.NaN));
            Assert.False(new DunaBrakingCommitment().Committed);
        }

        [Fact]
        public void GlobalTerrainAndPlanetRotationArePartOfProbe()
        {
            var result=DunaBrakingHandoff.Probe(new DunaBrakingHandoff.Vec(Radius+1000,0,0),
                new DunaBrakingHandoff.Vec(0,50,0),new DunaBrakingHandoff.Vec(0,0,50/(Radius+1000)),
                Mu,Radius,12,0);
            Assert.True(result.Safe);
            Assert.Equal(0,result.EndSpeed,8);
        }
    }
}
