using KSP.Localization;

namespace MuMech.Landing;

public class UntargetedDeorbit : AutopilotStep
{
	public UntargetedDeorbit(MechJebCore core)
		: base(core)
	{
	}

	public override AutopilotStep Drive(FlightCtrlState s)
	{
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		if (base.Orbit.PeA < -0.1 * base.MainBody.Radius)
		{
			Core.Thrust.TargetThrottle = 0f;
			return new FinalDescent(Core);
		}
		Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.ORBIT_HORIZONTAL, Core.Landing);
		Core.Thrust.TargetThrottle = ((Core.Attitude.attitudeAngleFromTarget() < 5.0) ? 1 : 0);
		base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status16");
		return this;
	}
}
