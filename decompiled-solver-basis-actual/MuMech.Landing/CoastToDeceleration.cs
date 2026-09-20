using System;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace MuMech.Landing;

public class CoastToDeceleration : AutopilotStep
{
	private const double RcsEnableCorrectionDv = 1.5;

	private const double RcsDisableCorrectionDv = 0.5;

	private const double MaximumRcsCorrectionDv = 3.0;

	private const double MaximumRcsCommandChange = 0.5;

	private bool _haveRcsCommand;

	private long _lastRcsPredictionVersion = -1L;

	private Vector3d _rcsCorrectionCommand;

	private bool _warpReady;

	public CoastToDeceleration(MechJebCore core)
		: base(core)
	{
	}

	public override AutopilotStep Drive(FlightCtrlState s)
	{
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		//IL_0082: Unknown result type (might be due to invalid IL or missing references)
		//IL_0087: Unknown result type (might be due to invalid IL or missing references)
		//IL_0095: Unknown result type (might be due to invalid IL or missing references)
		//IL_009a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0091: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_012d: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Landing.PredictionReady)
		{
			return this;
		}
		if (!Core.Landing.RCSAdjustment)
		{
			return this;
		}
		if (Core.Landing.PredictionVersion != _lastRcsPredictionVersion)
		{
			_lastRcsPredictionVersion = Core.Landing.PredictionVersion;
			Vector3d val = LimitMagnitude(Core.Landing.ComputeCourseCorrection(allowPrograde: true), 3.0);
			_rcsCorrectionCommand = (_haveRcsCommand ? MoveTowards(_rcsCorrectionCommand, val, 0.5) : val);
			_haveRcsCommand = true;
		}
		if (!_haveRcsCommand)
		{
			return this;
		}
		if (((Vector3d)(ref _rcsCorrectionCommand)).magnitude > 1.5)
		{
			Core.RCS.Enabled = true;
		}
		else if (((Vector3d)(ref _rcsCorrectionCommand)).magnitude < 0.5)
		{
			Core.RCS.Enabled = false;
		}
		if (Core.RCS.Enabled)
		{
			Core.RCS.SetWorldVelocityError(_rcsCorrectionCommand);
		}
		return this;
	}

	private static Vector3d LimitMagnitude(Vector3d vector, double maximumMagnitude)
	{
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		double magnitude = ((Vector3d)(ref vector)).magnitude;
		if (!(magnitude > maximumMagnitude))
		{
			return vector;
		}
		return vector * (maximumMagnitude / magnitude);
	}

	private static Vector3d MoveTowards(Vector3d current, Vector3d target, double maximumChange)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = target - current;
		double magnitude = ((Vector3d)(ref val)).magnitude;
		if (!(magnitude > maximumChange))
		{
			return target;
		}
		return current + val * (maximumChange / magnitude);
	}

	public override AutopilotStep OnFixedUpdate()
	{
		//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_0190: Unknown result type (might be due to invalid IL or missing references)
		//IL_0195: Unknown result type (might be due to invalid IL or missing references)
		//IL_035e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0363: Unknown result type (might be due to invalid IL or missing references)
		//IL_02fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0302: Unknown result type (might be due to invalid IL or missing references)
		//IL_0307: Unknown result type (might be due to invalid IL or missing references)
		//IL_0309: Unknown result type (might be due to invalid IL or missing references)
		//IL_0318: Unknown result type (might be due to invalid IL or missing references)
		//IL_031d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0322: Unknown result type (might be due to invalid IL or missing references)
		//IL_0327: Unknown result type (might be due to invalid IL or missing references)
		//IL_032b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0330: Unknown result type (might be due to invalid IL or missing references)
		//IL_033d: Unknown result type (might be due to invalid IL or missing references)
		Core.Thrust.TargetThrottle = 0f;
		if (Core.Landing.DeployChutes && Core.Landing.ParachutesDeployable())
		{
			Core.Landing.ControlParachutes();
		}
		double num = Core.Landing.MaxAllowedSpeed();
		if (base.VesselState.SpeedSurface > 0.9 * num)
		{
			Core.Warp.MinimumWarp();
			if (Core.Landing.RCSAdjustment)
			{
				Core.RCS.Enabled = false;
			}
			return new DecelerationBurn(Core);
		}
		base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status1");
		if (Core.Landing.LandAtTarget)
		{
			if (Vector3d.Distance(Core.Target.GetPositionTargetPosition(), Core.Landing.LandingSite) > 1000.0)
			{
				if (!base.VesselState.ParachuteDeployed && base.VesselState.DragAcceleration <= 0.1)
				{
					Core.Warp.MinimumWarp();
					if (Core.Landing.RCSAdjustment)
					{
						Core.RCS.Enabled = false;
					}
					return new CourseCorrection(Core);
				}
			}
			else
			{
				Vector3d val = Core.Landing.ComputeCourseCorrection(allowPrograde: true);
				base.Status = base.Status + "\n" + Localizer.Format("#MechJeb_LandingGuidance_Status2", new string[1] { ((Vector3d)(ref val)).magnitude.ToString("F3") });
			}
		}
		if (base.VesselState.AltitudeASL < Core.Landing.DecelerationEndAltitude() + 5.0)
		{
			Core.Warp.MinimumWarp();
			if (Core.Landing.RCSAdjustment)
			{
				Core.RCS.Enabled = false;
			}
			return new DecelerationBurn(Core);
		}
		if (Core.Attitude.attitudeAngleFromTarget() < 1.0)
		{
			_warpReady = true;
		}
		if (Core.Attitude.attitudeAngleFromTarget() > 5.0)
		{
			_warpReady = false;
		}
		if (Core.Landing.PredictionReady)
		{
			if (base.VesselState.DragAcceleration < 0.01)
			{
				double ut = (Core.Landing.Prediction.Trajectory.Any() ? Core.Landing.Prediction.Trajectory.First().UT : base.VesselState.Time);
				Vector3d val2 = -base.Orbit.WorldOrbitalVelocityAtUT(ut);
				val2 += base.MainBody.getRFrmVel(base.Orbit.WorldPositionAtUT(ut));
				val2 = ((Vector3d)(ref val2)).normalized;
				Core.Attitude.attitudeTo(val2, AttitudeReference.INERTIAL, Core.Landing);
			}
			else
			{
				Core.Attitude.attitudeTo(Vector3d.op_Implicit(Vector3.back), AttitudeReference.SURFACE_VELOCITY, Core.Landing);
			}
		}
		if (_warpReady && Core.Node.Autowarp)
		{
			double num2 = Math.Max(Math.Abs(base.VesselState.SpeedVertical), base.VesselState.LocalGravity * 5.0);
			Core.Warp.WarpRegularAtRate((float)(base.VesselState.AltitudeASL / (10.0 * num2)));
		}
		else
		{
			Core.Warp.MinimumWarp();
		}
		return this;
	}
}
