using System;
using D=MuMech.Landing.AtmosphericLandingDynamics;

namespace MuMech.Landing
{
    // The normal Land at target button arms this step. It owns the same
    // actuators as the landing module, but holds zero thrust while planning.
    // The main-thread terrain and engagement gates still run before the
    // worker's numerical plan can become an active controller.
    internal sealed class DunaIntegratedPlanning : AutopilotStep
    {
        private readonly DunaIntegratedPreview preview=new DunaIntegratedPreview();
        private D.V pointing;
        private double nextEngineCheck=double.NegativeInfinity;
        internal DunaIntegratedPlanning(MechJebCore core):base(core){}
        internal bool Start(out string reason)
        {
            if(!preview.Start(Core,out reason))return false;
            pointing=preview.Input.Initial.Forward;
            Status="Computing integrated "+(preview.Input.Strategic?"orbital landing":"descent");
            return true;
        }
        internal void Cancel()=>preview.Cancel();
        internal string ForecastText()=>Status;
        private bool Owns()=>Core.Landing.Enabled&&DunaLandingAdmission.OwnsActuators(
            Core.Attitude.Users.Count,Core.Attitude.Users.Contains(Core.Landing),
            Core.Thrust.Users.Count,Core.Thrust.Users.Contains(Core.Thrust),
            Core.Thrust.Users.Contains(Core.Landing),true);
        private AutopilotStep Finish(string reason)
        {
            Core.Landing.EndIntegratedDescent(reason);
            return null;
        }
        public override AutopilotStep Drive(FlightCtrlState controls)
        {
            try
            {
                if(!Owns())return Finish("planning actuator ownership changed");
                if(Vessel.LandedOrSplashed)return Finish("contact during planning");
                string reason=DunaIntegratedPreview.LiveCompatibility(Core,preview.Input,false);
                if(reason!=null)return Finish(reason);
                Core.Thrust.Tmode=MechJebModuleThrustController.TMode.OFF;
                Core.Thrust.TransKillH=false;
                Core.Attitude.attitudeTo(DunaIntegratedPreview.WorldAt(Vessel.mainBody,preview.Input.Frame,pointing,VesselState.Time),AttitudeReference.INERTIAL,Core.Landing);
                Core.Thrust.RequestActiveThrottle(0,false,true);
                return this;
            }
            catch(Exception ex){return Finish("planning control exception: "+ex.Message);}
        }
        public override AutopilotStep OnFixedUpdate()
        {
            try
            {
                bool owners=Owns();
                bool checkEngines=VesselState.Time>=nextEngineCheck;
                if(checkEngines)nextEngineCheck=VesselState.Time+1;
                string compatibility=DunaIntegratedPreview.LiveCompatibility(Core,preview.Input,checkEngines);
                if(!owners||compatibility!=null)return Finish(compatibility??"planning actuator ownership changed");
                Status=preview.Poll(Core);
                bool admitted=false;
                string reason=null;
                DunaIntegratedPreview.Snapshot snapshot=null;
                AtmosphericPlanTracker tracker=null;
                if(preview.ExecutionAvailable)
                    admitted=preview.TryTakePlan(Core,out snapshot,out tracker,out reason,true);
                switch(DunaLandingAdmission.Next(true,owners,preview.Running,preview.ExecutionAvailable,preview.Finished,admitted))
                {
                    case DunaLandingAdmission.Progress.Waiting:return this;
                    case DunaLandingAdmission.Progress.Engage:
                        UnityEngine.Debug.Log("[DunaIntegratedDescent] auto-engaged from Land at target; mode="+
                            (snapshot.Strategic?"orbit":"descent")+" miss="+tracker.TargetMiss+
                            " contactUT="+tracker.EndUT+" reserveKg="+tracker.ReserveMass);
                        Cancel();
                        return new DunaIntegratedDescent(Core,snapshot,tracker);
                    default:return Finish(reason??Status);
                }
            }
            catch(Exception ex){return Finish("planning exception: "+ex.Message);}
        }
    }
}
