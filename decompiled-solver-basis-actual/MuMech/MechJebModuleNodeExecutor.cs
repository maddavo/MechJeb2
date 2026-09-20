using System;
using MechJebLib.FuelFlowSimulation;
using MechJebLib.Utils;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech;

public class MechJebModuleNodeExecutor : ComputerModule
{
	public enum Modes
	{
		ONE_NODE,
		ALL_NODES
	}

	public enum States
	{
		INITIAL_WARP,
		ALIGNING,
		WARPING,
		LEAD,
		BURN,
		IDLE
	}

	[Persistent(pass = 4)]
	public bool Autowarp = true;

	[Persistent(pass = 4)]
	public readonly EditableDouble LeadTime = new EditableDouble(3.0);

	[Persistent(pass = 4)]
	public readonly EditableDouble InitialWarpLeadTime = new EditableDouble(600.0);

	[Persistent(pass = 4)]
	public readonly EditableDouble AlignedToleranceDegrees = new EditableDouble(1.0);

	[Persistent(pass = 4)]
	public readonly EditableDouble WarpAlignedToleranceDegrees = new EditableDouble(10.0);

	public bool RCSOnly;

	[Persistent(pass = 4)]
	public bool KillRollRotation = true;

	public Modes Mode;

	public States State = States.IDLE;

	private double _dvLeft;

	private Vector3d _direction;

	private double _ignitionUT;

	private double _timeToBurn;

	private double _ullageUntil;

	private static bool _isLoadedRealFuels => ReflectionUtils.IsAssemblyLoaded("RealFuels");

	private static bool _isLoadedPrincipia => ReflectionUtils.IsAssemblyLoaded("principia.ksp_plugin_adapter");

	private Vector3d _worldDirection => Planetarium.fetch.rotation * _direction;

	private bool _hasNodes => base.Vessel.patchedConicSolver.maneuverNodes.Count > 0;

	[ValueInfoItem("#MechJeb_NodeBurnLength", InfoItem.Category.Thrust)]
	public string NextNodeBurnTime()
	{
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		if (!base.Vessel.patchedConicsUnlocked())
		{
			return "-";
		}
		double dv;
		if (_isLoadedPrincipia && _dvLeft > 0.0)
		{
			dv = _dvLeft;
		}
		else
		{
			if (base.Vessel.patchedConicSolver.maneuverNodes.Count == 0)
			{
				return "-";
			}
			Vector3d burnVector = base.Vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(base.Orbit);
			dv = ((Vector3d)(ref burnVector)).magnitude;
		}
		double halfBurnTime;
		double spoolupTime;
		return GuiUtils.TimeToDHMS(BurnTime(dv, out halfBurnTime, out spoolupTime));
	}

	[ValueInfoItem("#MechJeb_NodeBurnCountdown", InfoItem.Category.Thrust)]
	public string NextNodeCountdown()
	{
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		if (!base.Vessel.patchedConicsUnlocked() || base.Vessel.patchedConicSolver.maneuverNodes.Count == 0)
		{
			return "-";
		}
		ManeuverNode val = base.Vessel.patchedConicSolver.maneuverNodes[0];
		double num;
		if (!_isLoadedPrincipia)
		{
			Vector3d burnVector = val.GetBurnVector(base.Orbit);
			num = ((Vector3d)(ref burnVector)).magnitude;
		}
		else
		{
			num = _dvLeft;
		}
		double dv = num;
		double uT = val.UT;
		BurnTime(dv, out var halfBurnTime, out var spoolupTime);
		uT = ((!_isLoadedPrincipia) ? (uT - halfBurnTime) : (uT - spoolupTime));
		return GuiUtils.TimeToDHMS(uT - base.VesselState.Time);
	}

	public void ExecuteOneNode(object controller)
	{
		Users.Add(controller);
		Mode = Modes.ONE_NODE;
		Init();
	}

	public void ExecuteAllNodes(object controller)
	{
		Users.Add(controller);
		Mode = Modes.ALL_NODES;
		Init();
	}

	private void Init()
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		_direction = Vector3d.zero;
		Vector3d burnVector = base.Vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(base.Orbit);
		_dvLeft = ((Vector3d)(ref burnVector)).magnitude;
		Core.Thrust.Users.Add(this);
		Core.Attitude.Users.Add(this);
		TransitionTo(States.INITIAL_WARP);
	}

	public void Abort()
	{
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		Core.Warp.MinimumWarp();
		Core.Thrust.ThrustOff();
		Core.Attitude.attitudeDeactivate();
		Users.Clear();
		_direction = Vector3d.zero;
		_dvLeft = 0.0;
		State = States.IDLE;
	}

	protected override void OnModuleEnabled()
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		State = States.IDLE;
		_direction = Vector3d.zero;
		_dvLeft = 0.0;
	}

	protected override void OnModuleDisabled()
	{
		Core.Attitude.attitudeDeactivate();
		Core.Thrust.ThrustOff();
		Core.Thrust.Users.Remove(this);
		State = States.IDLE;
		_dvLeft = 0.0;
	}

	public override void Drive(FlightCtrlState s)
	{
		DoRCS(s);
	}

	private void DoRCS(FlightCtrlState s)
	{
		if (State == States.IDLE)
		{
			return;
		}
		if (State == States.BURN && RCSOnly)
		{
			base.Vessel.ActionGroups.SetGroup((KSPActionGroup)8, true);
			s.Z = -1f;
		}
		else
		{
			if (State != States.LEAD || RCSOnly || !Core.Thrust.AutoRCSUllaging || !_isLoadedRealFuels)
			{
				return;
			}
			if (base.VesselState.Time >= _ignitionUT - 0.25)
			{
				_ullageUntil = _ignitionUT;
			}
			if (base.VesselState.LowestUllage >= 1.0 && base.VesselState.Time > _ullageUntil)
			{
				return;
			}
			if (base.VesselState.LowestUllage < 1.0)
			{
				_ullageUntil = base.VesselState.Time + 0.25;
			}
			if (base.Vessel.hasEnabledRCSModules())
			{
				if (!base.Vessel.ActionGroups[(KSPActionGroup)8])
				{
					base.Vessel.ActionGroups.SetGroup((KSPActionGroup)8, true);
				}
				if (Aligned())
				{
					s.Z = -1f;
				}
			}
		}
	}

	public override void OnFixedUpdate()
	{
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		if (!base.Vessel.patchedConicsUnlocked() || (!_isLoadedPrincipia && !_hasNodes) || State == States.IDLE)
		{
			Abort();
			return;
		}
		_direction = NextDirection();
		_ignitionUT = CalculateIgnitionUT();
		_timeToBurn = _ignitionUT - base.VesselState.Time;
		UpdateState();
		TickState();
	}

	private void UpdateState()
	{
		States states = DetermineState(State);
		if (states != State)
		{
			TransitionTo(states);
		}
	}

	private States DetermineState(States desired)
	{
		switch (desired)
		{
		case States.IDLE:
			return States.IDLE;
		default:
			if (!(base.VesselState.Time >= _ignitionUT) || !Aligned())
			{
				break;
			}
			goto case States.BURN;
		case States.BURN:
			return States.BURN;
		}
		if (base.VesselState.Time >= _ignitionUT - (double)LeadTime)
		{
			return States.LEAD;
		}
		if (!Autowarp)
		{
			return States.ALIGNING;
		}
		if (MuUtils.PhysicsRunning() ? AlignedAndSettled() : (AngleFromDirection() < Statics.Deg2Rad((double)WarpAlignedToleranceDegrees)))
		{
			return States.WARPING;
		}
		if (_timeToBurn > (double)InitialWarpLeadTime)
		{
			return States.INITIAL_WARP;
		}
		return States.ALIGNING;
	}

	private void TransitionTo(States next)
	{
		State = next;
		switch (next)
		{
		case States.INITIAL_WARP:
			OnEnterInitialWarp();
			break;
		case States.ALIGNING:
			OnEnterAligning();
			break;
		case States.WARPING:
			OnEnterWarping();
			break;
		case States.LEAD:
			OnEnterLead();
			break;
		case States.BURN:
			OnEnterBurn();
			break;
		case States.IDLE:
			OnEnterIdle();
			break;
		}
	}

	private void TickState()
	{
		Core.Attitude.SetAxisControl(pitch: false, yaw: false, roll: false);
		switch (State)
		{
		case States.INITIAL_WARP:
			TickInitialWarp();
			break;
		case States.ALIGNING:
			TickAligning();
			break;
		case States.WARPING:
			TickWarping();
			break;
		case States.LEAD:
			TickLead();
			break;
		case States.BURN:
			TickBurn();
			break;
		}
	}

	private void OnEnterInitialWarp()
	{
		Core.Thrust.ThrustOff();
	}

	private void TickInitialWarp()
	{
		Core.Warp.WarpToUT(_ignitionUT - (double)InitialWarpLeadTime);
	}

	private void OnEnterAligning()
	{
		Core.Thrust.ThrustOff();
	}

	private void TickAligning()
	{
		if (Autowarp && !MuUtils.PhysicsRunning())
		{
			Core.Warp.MinimumWarp();
		}
		else
		{
			SetAttitude();
		}
	}

	private void OnEnterWarping()
	{
		Core.Thrust.ThrustOff();
	}

	private void TickWarping()
	{
		SetAttitude();
		Core.Warp.WarpToUT(_ignitionUT - (double)LeadTime);
	}

	private void OnEnterLead()
	{
		Core.Thrust.ThrustOff();
	}

	private void TickLead()
	{
		if (!MuUtils.PhysicsRunning())
		{
			Core.Warp.MinimumWarp();
			return;
		}
		SetAttitude();
		UpdateDvLeft();
	}

	private void OnEnterBurn()
	{
	}

	private void OnEnterIdle()
	{
	}

	private void TickBurn()
	{
		if (!MuUtils.PhysicsRunning())
		{
			Core.Warp.MinimumWarp();
			return;
		}
		SetAttitude();
		UpdateDvLeft();
		if (!ShouldTerminate() && !RCSOnly)
		{
			double timeConstant = ((_dvLeft > 10.0 || base.VesselState.MinThrustAcceleration > 0.25 * base.VesselState.MaxThrustAcceleration) ? 0.5 : 2.0);
			Core.Thrust.ThrustForDv(_dvLeft, timeConstant);
		}
	}

	private void SetAttitude()
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		Core.Attitude.attitudeTo(_worldDirection, AttitudeReference.INERTIAL_COT, this, KillRollRotation);
	}

	private bool ShouldTerminatePrincipia()
	{
		if (_dvLeft > 0.0)
		{
			return false;
		}
		if (Mode == Modes.ALL_NODES && base.Vessel.patchedConicSolver.maneuverNodes.Count > 0)
		{
			Init();
		}
		else
		{
			Abort();
		}
		return true;
	}

	private bool ShouldTerminateStock()
	{
		if (AngleFromNode() < Math.PI / 2.0)
		{
			return false;
		}
		base.Vessel.patchedConicSolver.maneuverNodes[0].RemoveSelf();
		if (Mode == Modes.ALL_NODES && base.Vessel.patchedConicSolver.maneuverNodes.Count > 0)
		{
			Init();
		}
		else
		{
			Abort();
		}
		return true;
	}

	private bool ShouldTerminate()
	{
		if (!_isLoadedPrincipia)
		{
			return ShouldTerminateStock();
		}
		return ShouldTerminatePrincipia();
	}

	private bool Aligned()
	{
		return AngleFromDirection() < Statics.Deg2Rad((double)AlignedToleranceDegrees);
	}

	private bool AlignedAndSettled()
	{
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		if (Aligned())
		{
			Vector3 val = Vector3.Scale(((PartModule)Core).vessel.angularVelocity, new Vector3(1f, KillRollRotation ? 1f : 0f, 1f));
			return (double)((Vector3)(ref val)).magnitude < 0.001;
		}
		return false;
	}

	private double AngleFromNode()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		Vector3d forward = base.VesselState.Forward;
		Vector3d burnVector = base.Vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(base.Orbit);
		Vector3d normalized = ((Vector3d)(ref burnVector)).normalized;
		return Statics.SafeAcos(Vector3d.Dot(forward, normalized));
	}

	private double AngleFromDirection()
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		Vector3d forward = base.VesselState.Forward;
		Vector3d worldDirection = _worldDirection;
		Vector3d normalized = ((Vector3d)(ref worldDirection)).normalized;
		return Statics.SafeAcos(Vector3d.Dot(forward, normalized));
	}

	private ManeuverNode SafeCurrentPrincipiaNode()
	{
		if (!_hasNodes)
		{
			return null;
		}
		ManeuverNode val = base.Vessel.patchedConicSolver.maneuverNodes[0];
		if (State == States.BURN && val.UT > base.VesselState.Time)
		{
			return null;
		}
		return val;
	}

	private Vector3d NextDirection()
	{
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_004c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0089: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_0062: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		QuaternionD val = QuaternionD.Inverse(Planetarium.fetch.rotation);
		Vector3d burnVector;
		if (_direction == Vector3d.zero)
		{
			burnVector = base.Vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(base.Orbit);
			return val * ((Vector3d)(ref burnVector)).normalized;
		}
		if (_isLoadedPrincipia && SafeCurrentPrincipiaNode() == null)
		{
			return _direction;
		}
		if (!_isLoadedPrincipia && _dvLeft < base.VesselState.MaxThrustAcceleration)
		{
			return _direction;
		}
		burnVector = base.Vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(base.Orbit);
		return val * ((Vector3d)(ref burnVector)).normalized;
	}

	private double CalculateIgnitionUT()
	{
		BurnTime(_dvLeft, out var halfBurnTime, out var spoolupTime);
		if (_isLoadedPrincipia)
		{
			if (SafeCurrentPrincipiaNode() == null)
			{
				return -1.0;
			}
			return base.Vessel.patchedConicSolver.maneuverNodes[0].UT - spoolupTime;
		}
		return base.Vessel.patchedConicSolver.maneuverNodes[0].UT - halfBurnTime;
	}

	private void UpdateDvLeft()
	{
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		if (!_isLoadedPrincipia)
		{
			Vector3d burnVector = base.Vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(base.Orbit);
			_dvLeft = ((Vector3d)(ref burnVector)).magnitude;
		}
		else
		{
			DecrementDvLeft();
		}
	}

	private void DecrementDvLeft()
	{
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		if (MuUtils.PhysicsRunning())
		{
			Vector3d val = (base.Vessel.acceleration_immediate - base.Vessel.graviticAcceleration) * (double)TimeWarp.fixedDeltaTime;
			_dvLeft -= Vector3d.Dot(val, _worldDirection);
		}
	}

	private double BurnTime(double dv, out double halfBurnTime, out double spoolupTime)
	{
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_006c: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0110: Unknown result type (might be due to invalid IL or missing references)
		//IL_0126: Unknown result type (might be due to invalid IL or missing references)
		//IL_0130: Unknown result type (might be due to invalid IL or missing references)
		//IL_0137: Unknown result type (might be due to invalid IL or missing references)
		//IL_013e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0156: Unknown result type (might be due to invalid IL or missing references)
		//IL_015d: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c4: Unknown result type (might be due to invalid IL or missing references)
		double num = dv;
		double num2 = dv / 2.0;
		double num3 = 0.0;
		halfBurnTime = 0.0;
		spoolupTime = 0.0;
		MechJebModuleStageStats computerModule = Core.GetComputerModule<MechJebModuleStageStats>();
		computerModule.RequestUpdate();
		double num4 = 0.0;
		int num5 = computerModule.VacStats.Count - 1;
		while (num5 >= 0 && num > 0.0)
		{
			FuelStats val = computerModule.VacStats[num5];
			if (val.DeltaV <= 0.0 || val.Thrust <= 0.0)
			{
				if (Core.Staging.Enabled)
				{
					if (num3 - num4 < (double)Core.Staging.AutostagePreDelay && num5 != computerModule.VacStats.Count - 1)
					{
						num3 += (double)Core.Staging.AutostagePreDelay - (num3 - num4);
					}
					num3 += (double)Core.Staging.AutostagePreDelay;
					num4 = num3;
				}
			}
			else
			{
				double num6 = Math.Min(val.DeltaV, num);
				num -= num6;
				double num7 = num6 / val.DeltaV;
				double num8 = val.StartMass / Math.Exp(Math.Log(val.StartMass / val.EndMass) * num7);
				double num9 = val.Thrust / ((val.StartMass + num8) / 2.0);
				if (num5 == computerModule.VacStats.Count - 1)
				{
					num9 *= (double)base.VesselState.ThrottleFixedLimit;
				}
				halfBurnTime += Math.Min(num2, num6) / num9;
				num2 = Math.Max(0.0, num2 - num6);
				num3 += num6 / num9;
				spoolupTime += val.SpoolUpTime;
			}
			num5--;
		}
		if (double.IsInfinity(halfBurnTime))
		{
			halfBurnTime = 0.0;
		}
		if (double.IsInfinity(num3))
		{
			num3 = 0.0;
		}
		if (spoolupTime > 0.0 && num3 > 0.0)
		{
			if (num3 < spoolupTime * 0.5)
			{
				spoolupTime = num3 / (spoolupTime * 0.5);
				num3 += spoolupTime;
				halfBurnTime += spoolupTime;
			}
			else
			{
				num3 += spoolupTime;
				halfBurnTime += spoolupTime;
			}
		}
		return num3;
	}

	public MechJebModuleNodeExecutor(MechJebCore core)
		: base(core)
	{
	}
}
