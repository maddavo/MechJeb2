using MechJebLib.Control;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleHoverslamAutopilot : ComputerModule
{
	private enum State
	{
		Init,
		Align,
		Coast,
		Burn,
		Vertical,
		Finished
	}

	[EditableInfoItem("#MechJeb_HoverslamIgnitionLead", InfoItem.Category.Hoverslam, width = 50f, rightLabel = "s", expandWidth = true, tooltip = "#MechJeb_HoverslamIgnitionLead_tooltip")]
	[Persistent(pass = 6)]
	public readonly EditableDouble IgnitionLead = new EditableDouble(0.0);

	[EditableInfoItem("#MechJeb_HoverslamTouchdownSpeed", InfoItem.Category.Hoverslam, width = 50f, rightLabel = "m/s", expandWidth = true, tooltip = "#MechJeb_HoverslamTouchdownSpeed_tooltip")]
	[Persistent(pass = 6)]
	public readonly EditableDouble TouchdownSpeed = new EditableDouble(0.0);

	[ToggleInfoItem("#MechJeb_HoverslamAutoWarp", InfoItem.Category.Hoverslam, tooltip = "#MechJeb_HoverslamAutoWarp_tooltip")]
	[Persistent(pass = 6)]
	public bool AutoWarp = true;

	[ToggleInfoItem("#MechJeb_HoverslamHoldUpright", InfoItem.Category.Hoverslam, tooltip = "#MechJeb_HoverslamHoldUpright_tooltip")]
	[Persistent(pass = 6)]
	public bool HoldUpright;

	private const double DEFAULT_MIN_ON_TIME = 0.5;

	[EditableInfoItem("#MechJeb_HoverslamPWMPulseWidth", InfoItem.Category.Hoverslam, width = 50f, rightLabel = "s", expandWidth = true, tooltip = "#MechJeb_HoverslamPWMTimeWidth_tooltip")]
	[Persistent(pass = 6)]
	public readonly EditableDouble PWMPulseWidth = new EditableDouble(0.5);

	private State _state;

	private Vector3d _lastAdjV;

	private Vector3d _adjV;

	private readonly DeltaSigmaThrottleModulator _pwm = new DeltaSigmaThrottleModulator(0.02, 0.5);

	[ValueInfoItem("#MechJeb_HoverslamState", InfoItem.Category.Hoverslam, tooltip = "#MechJeb_HoverslamState_tooltip")]
	public string HoverslamState
	{
		get
		{
			if (base.Enabled)
			{
				return _state.ToString();
			}
			return "Disabled";
		}
	}

	public MechJebModuleHoverslamAutopilot(MechJebCore core)
		: base(core)
	{
	}//IL_0056: Unknown result type (might be due to invalid IL or missing references)
	//IL_0060: Expected O, but got Unknown


	[ActionInfoItem("#MechJeb_HoverslamEngage", InfoItem.Category.Hoverslam, tooltip = "#MechJeb_HoverslamEngage_tooltip")]
	public void ToggleEnabled()
	{
		base.Enabled = !base.Enabled;
	}

	protected override void OnModuleEnabled()
	{
		Reset();
		Core.Thrust.Users.Add(this);
		Core.Attitude.Users.Add(this);
		TransitionTo(DetermineState(State.Align));
	}

	protected override void OnModuleDisabled()
	{
		Reset();
		Core.Warp.MinimumWarp(instant: true);
		Core.Thrust.ThrustOff();
		Core.Thrust.Users.Remove(this);
		Core.Attitude.Users.Remove(this);
	}

	private void Reset()
	{
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		_lastAdjV = new Vector3d(double.NaN, double.NaN, double.NaN);
	}

	public override void OnFixedUpdate()
	{
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		UpdateAdjV();
		UpdateState();
		TickState();
		_lastAdjV = _adjV;
	}

	private void UpdateAdjV()
	{
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		_adjV = base.VesselState.SurfaceVelocity + Core.Hoverslam.FinalDescentSpeed * base.VesselState.Up;
	}

	private bool AlignedForBurn()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		if (Statics.SafeAcos(Vector3d.Dot(base.VesselState.Forward, Core.Hoverslam.IgnitionAttitude)) < Statics.Deg2Rad(1.0))
		{
			return (double)((Vector3)(ref base.Vessel.angularVelocity)).magnitude < 0.001;
		}
		return false;
	}

	private State DetermineState(State desired)
	{
		//IL_00ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		if (!Statics.IsFinite(Core.Hoverslam.IgnitionUT))
		{
			return State.Init;
		}
		if (desired == State.Init)
		{
			desired = State.Align;
		}
		if (desired == State.Align && AlignedForBurn())
		{
			desired = State.Coast;
		}
		double ignitionCountdown = Core.Hoverslam.IgnitionCountdown;
		if ((desired == State.Coast || desired == State.Align) && Statics.IsFinite(ignitionCountdown) && ignitionCountdown <= (double)Time.fixedDeltaTime + (double)IgnitionLead)
		{
			desired = State.Burn;
		}
		if (desired == State.Burn && !double.IsNaN(((Vector3d)(ref _lastAdjV))[0]) && Vector3d.Angle(_lastAdjV, _adjV) > 10.0)
		{
			desired = State.Vertical;
		}
		if (desired == State.Burn || desired == State.Vertical)
		{
			if (Vector3d.Dot(base.VesselState.SurfaceVelocity, base.VesselState.Up) >= 0.0)
			{
				desired = State.Finished;
			}
			if (!base.Vessel.VesselOffGround())
			{
				desired = State.Finished;
			}
		}
		return desired;
	}

	private void UpdateState()
	{
		State state = DetermineState(_state);
		if (state != _state)
		{
			TransitionTo(state);
		}
	}

	private void TransitionTo(State next)
	{
		_state = next;
		switch (next)
		{
		case State.Init:
			OnEnterInit();
			break;
		case State.Align:
			OnEnterAligning();
			break;
		case State.Coast:
			OnEnterCoast();
			break;
		case State.Burn:
			OnEnterBurn();
			break;
		case State.Vertical:
			OnEnterFinalDescent();
			break;
		case State.Finished:
			OnEnterFinished();
			break;
		}
	}

	private void TickState()
	{
		switch (_state)
		{
		case State.Init:
			TickInit();
			break;
		case State.Align:
			TickAligning();
			break;
		case State.Coast:
			TickCoast();
			break;
		case State.Burn:
			TickBurn();
			break;
		case State.Vertical:
			TickFinalDescent();
			break;
		case State.Finished:
			TickFinished();
			break;
		}
	}

	private void OnEnterInit()
	{
		Core.Thrust.ThrustOff();
	}

	private void TickInit()
	{
	}

	private void OnEnterAligning()
	{
		Core.Thrust.ThrustOff();
	}

	private void TickAligning()
	{
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		Core.Attitude.attitudeTo(Core.Hoverslam.IgnitionAttitude, AttitudeReference.INERTIAL_COT, this);
		if (!MuUtils.PhysicsRunning() && base.VesselState.Time + (double)TimeWarp.fixedDeltaTime > Core.Hoverslam.IgnitionUT)
		{
			Core.Warp.MinimumWarp(instant: true);
		}
	}

	private void OnEnterCoast()
	{
		Core.Thrust.ThrustOff();
	}

	private void TickCoast()
	{
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		Core.Attitude.attitudeTo(Core.Hoverslam.IgnitionAttitude, AttitudeReference.INERTIAL_COT, this);
		if (AutoWarp)
		{
			Core.Warp.WarpToUT(Core.Hoverslam.IgnitionUT);
		}
		else if (!MuUtils.PhysicsRunning() && base.VesselState.Time + (double)TimeWarp.fixedDeltaTime > Core.Hoverslam.IgnitionUT)
		{
			Core.Warp.MinimumWarp(instant: true);
		}
	}

	private void OnEnterBurn()
	{
		Core.Thrust.TargetThrottle = 1f;
	}

	private void TickBurn()
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		Core.Attitude.attitudeTo(-_adjV, AttitudeReference.INERTIAL_COT, this);
	}

	private void OnEnterFinalDescent()
	{
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		_pwm.Reset();
		Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, this);
	}

	private void TickFinalDescent()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		if (Vector3d.Dot(base.VesselState.SurfaceVelocity, base.VesselState.Up) >= -1.0)
		{
			Core.Attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, this);
		}
		double sqrMagnitude = ((Vector3d)(ref base.VesselState.SurfaceVelocity)).sqrMagnitude;
		double magnitude = ((Vector3d)(ref base.Vessel.graviticAcceleration)).magnitude;
		double altitudeBottom = base.VesselState.AltitudeBottom;
		double num = TouchdownSpeed;
		double num2 = magnitude + 0.5 * (sqrMagnitude - num * num) / altitudeBottom;
		_pwm.MinOnTime = PWMPulseWidth;
		_pwm.MinOffTime = TimeWarp.fixedDeltaTime;
		Core.Thrust.TargetThrottle = _pwm.ThrottleCommand(num2, base.VesselState.MinThrustAcceleration, base.VesselState.MaxThrustAcceleration, (double)TimeWarp.fixedDeltaTime);
	}

	private void OnEnterFinished()
	{
		if (HoldUpright)
		{
			Core.SmartASS.mode = MechJebModuleSmartASS.Mode.SURFACE;
			Core.SmartASS.target = MechJebModuleSmartASS.Target.VERTICAL_PLUS;
			Core.SmartASS.Engage();
		}
		base.Enabled = false;
	}

	private void TickFinished()
	{
	}
}
