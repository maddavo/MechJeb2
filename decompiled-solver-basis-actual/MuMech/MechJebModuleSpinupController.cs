using System;
using UnityEngine;

namespace MuMech;

public class MechJebModuleSpinupController : ComputerModule
{
	private enum SpinupState
	{
		INITIALIZED,
		STARTING,
		STABILIZING,
		SPINUP,
		FINISHED
	}

	public double RollAngularVelocity;

	private SpinupState _state;

	private double _startTime;

	public MechJebModuleSpinupController(MechJebCore core)
		: base(core)
	{
	}

	protected override void OnModuleEnabled()
	{
		_state = SpinupState.INITIALIZED;
		_startTime = Math.Max(base.VesselState.Time, _startTime);
		Core.Attitude.Users.Add(this);
	}

	protected override void OnModuleDisabled()
	{
		_state = SpinupState.FINISHED;
		Core.Attitude.SetOmegaTarget();
		Core.Attitude.SetActuationControl();
		Core.Staging.AutoStageLimitRemove(this);
		Core.Attitude.Users.Remove(this);
		base.OnModuleDisabled();
	}

	public override void OnStart(StartState state)
	{
		GameEvents.onStageActivate.Add((OnEvent<int>)HandleStageEvent);
	}

	public override void OnDestroy()
	{
		GameEvents.onStageActivate.Remove((OnEvent<int>)HandleStageEvent);
	}

	private void HandleStageEvent(int data)
	{
		_startTime = base.VesselState.Time + 1.0;
	}

	public override void Drive(FlightCtrlState s)
	{
		if (_state == SpinupState.INITIALIZED)
		{
			return;
		}
		Core.Staging.AutoStageLimitRequest(base.Vessel.currentStage, this);
		if (!(base.VesselState.Time < _startTime))
		{
			if (_state == SpinupState.STARTING)
			{
				_state = SpinupState.STABILIZING;
			}
			if (base.Vessel.angularVelocityD.y / RollAngularVelocity >= 0.99)
			{
				base.Enabled = false;
			}
			if (!base.Vessel.ActionGroups[(KSPActionGroup)8])
			{
				base.Vessel.ActionGroups.SetGroup((KSPActionGroup)8, true);
			}
			if (_state != SpinupState.STABILIZING || (!(Core.Attitude.attitudeAngleFromTarget() > 1.0) && !((double)((Vector3)(ref ((PartModule)Core).vessel.angularVelocity)).magnitude > 0.001)))
			{
				_state = SpinupState.SPINUP;
				Core.Attitude.SetOmegaTarget(double.NaN, double.NaN, RollAngularVelocity);
				Core.Attitude.SetActuationControl(pitch: false, yaw: false);
			}
		}
	}

	public void AssertStart()
	{
		if (_state == SpinupState.INITIALIZED)
		{
			_state = SpinupState.STARTING;
		}
	}
}
