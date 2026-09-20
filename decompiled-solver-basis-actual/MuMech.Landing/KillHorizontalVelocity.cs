using KSP.Localization;
using UnityEngine;

namespace MuMech.Landing;

public class KillHorizontalVelocity : AutopilotStep
{
	private const double FinalDescentHorizontalSpeed = 1.0;

	public KillHorizontalVelocity(MechJebCore core)
		: base(core)
	{
	}

	public override AutopilotStep Drive(FlightCtrlState s)
	{
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_009f: Unknown result type (might be due to invalid IL or missing references)
		//IL_00aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0071: Unknown result type (might be due to invalid IL or missing references)
		//IL_0076: Unknown result type (might be due to invalid IL or missing references)
		//IL_0166: Unknown result type (might be due to invalid IL or missing references)
		//IL_0174: Unknown result type (might be due to invalid IL or missing references)
		//IL_0175: Unknown result type (might be due to invalid IL or missing references)
		//IL_017a: Unknown result type (might be due to invalid IL or missing references)
		//IL_017f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0183: Unknown result type (might be due to invalid IL or missing references)
		//IL_0188: Unknown result type (might be due to invalid IL or missing references)
		//IL_0195: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Landing.PredictionReady)
		{
			return this;
		}
		Vector3d val = Vector3d.Exclude(base.VesselState.Up, base.VesselState.Forward);
		Vector3d normalized = ((Vector3d)(ref val)).normalized;
		if (base.VesselState.SpeedSurfaceHorizontal <= 1.0)
		{
			Core.Thrust.RequestActiveThrottle(0f);
			Core.Attitude.attitudeTo(Vector3d.op_Implicit(Vector3.up), AttitudeReference.SURFACE_NORTH, Core.Landing);
			return new FinalDescent(Core);
		}
		double num = Vector3d.Dot(base.VesselState.SurfaceVelocity, base.VesselState.Up);
		double num2 = (0.0 - num) / 1.0;
		double num3 = 0.0 - base.VesselState.LocalGravity;
		double num4 = 0.0 - base.VesselState.LocalGravity + Vector3d.Dot(base.VesselState.Forward, base.VesselState.Up) * base.VesselState.MaxThrustAcceleration;
		if (num4 - num3 > 0.0)
		{
			Core.Thrust.RequestActiveThrottle(Mathf.Clamp((float)((num2 - num3) / (num4 - num3)), 0f, 1f));
		}
		else
		{
			Core.Thrust.RequestActiveThrottle(0f);
		}
		val = base.VesselState.Up + 0.2 * normalized;
		Vector3d normalized2 = ((Vector3d)(ref val)).normalized;
		Core.Attitude.attitudeTo(normalized2, AttitudeReference.INERTIAL, Core.Landing);
		base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status10");
		return this;
	}
}
