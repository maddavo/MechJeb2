using System;
using System.Collections.Generic;
using MuMech.AttitudeControllers;
using UnityEngine;

namespace MuMech;

public class MechJebModuleAttitudeController : ComputerModule
{
	private float timeCount;

	private Part lastReferencePart;

	public bool RCS_auto;

	private readonly bool attitudeRCScontrol = true;

	[Persistent(pass = 4)]
	[ValueInfoItem("#MechJeb_SteeringError", InfoItem.Category.Vessel, format = "F1", units = "º")]
	public readonly MovingAverage steeringError = new MovingAverage();

	public bool attitudeKILLROT;

	private bool attitudeChanged;

	private AttitudeReference _attitudeReference;

	private readonly List<BaseAttitudeController> _controllers = new List<BaseAttitudeController>();

	[Persistent(pass = 4)]
	public int activeController = 3;

	private QuaternionD _attitudeTarget = QuaternionD.identity;

	private readonly bool useSAS;

	private QuaternionD lastSAS;

	public double attitudeError;

	public Vector3d torque;

	public Vector3d inertia;

	public Vector3d AxisControl { get; private set; } = Vector3d.one;


	public Vector3d ActuationControl { get; private set; } = Vector3d.one;


	public Vector3d OmegaTarget { get; private set; } = new Vector3d(double.NaN, double.NaN, double.NaN);


	public BaseAttitudeController Controller { get; private set; }

	public AttitudeReference attitudeReference
	{
		get
		{
			return _attitudeReference;
		}
		private set
		{
			if (_attitudeReference != value)
			{
				_attitudeReference = value;
				attitudeChanged = true;
			}
		}
	}

	public QuaternionD attitudeTarget
	{
		get
		{
			//IL_0001: Unknown result type (might be due to invalid IL or missing references)
			return _attitudeTarget;
		}
		private set
		{
			//IL_0001: Unknown result type (might be due to invalid IL or missing references)
			//IL_0006: Unknown result type (might be due to invalid IL or missing references)
			//IL_000b: Unknown result type (might be due to invalid IL or missing references)
			//IL_0010: Unknown result type (might be due to invalid IL or missing references)
			//IL_0011: Unknown result type (might be due to invalid IL or missing references)
			//IL_0016: Unknown result type (might be due to invalid IL or missing references)
			//IL_0041: Unknown result type (might be due to invalid IL or missing references)
			//IL_0042: Unknown result type (might be due to invalid IL or missing references)
			if (Math.Abs(Vector3d.Angle(_attitudeTarget * Vector3d.forward, value * Vector3d.forward)) > 10.0)
			{
				SetAxisControl(pitch: true, yaw: true, roll: true);
				attitudeChanged = true;
			}
			_attitudeTarget = value;
		}
	}

	public QuaternionD RequestedAttitude { get; private set; } = QuaternionD.identity;


	public void SetActiveController(int i)
	{
		activeController = i;
		Controller = _controllers[activeController];
		Controller.OnStart();
	}

	protected override void OnModuleEnabled()
	{
		timeCount = 50f;
		SetAxisControl(pitch: true, yaw: true, roll: true);
		SetActuationControl();
		SetOmegaTarget();
		Controller.OnModuleEnabled();
	}

	public MechJebModuleAttitudeController(MechJebCore core)
		: base(core)
	{
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_006c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0071: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		Priority = 800;
		_controllers.Add(new MJAttitudeController(this));
		_controllers.Add(new KosAttitudeController(this));
		_controllers.Add(new HybridController(this));
		_controllers.Add(new BetterController(this));
		_controllers.Add(new LQRController(this));
		Controller = new BetterController(this);
	}

	protected override void OnModuleDisabled()
	{
		if (useSAS)
		{
			base.Part.vessel.ActionGroups.SetGroup((KSPActionGroup)16, false);
		}
		Controller.OnModuleDisabled();
	}

	public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnLoad(local, type, global);
		foreach (BaseAttitudeController controller in _controllers)
		{
			controller.OnLoad(local, type, global);
		}
	}

	public override void OnStart(StartState state)
	{
		SetActiveController(activeController);
	}

	public override void OnSave(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnSave(local, type, global);
		foreach (BaseAttitudeController controller in _controllers)
		{
			controller.OnSave(local, type, global);
		}
	}

	public void SetAxisControl(bool pitch, bool yaw, bool roll)
	{
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		AxisControl = new Vector3d((double)(pitch ? 1 : 0), (double)(roll ? 1 : 0), (double)(yaw ? 1 : 0));
	}

	public void SetActuationControl(bool pitch = true, bool yaw = true, bool roll = true)
	{
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		ActuationControl = new Vector3d((double)(pitch ? 1 : 0), (double)(roll ? 1 : 0), (double)(yaw ? 1 : 0));
	}

	public void SetOmegaTarget(double pitch = double.NaN, double yaw = double.NaN, double roll = double.NaN)
	{
		//IL_0004: Unknown result type (might be due to invalid IL or missing references)
		OmegaTarget = new Vector3d(pitch, roll, yaw);
	}

	public QuaternionD attitudeGetReferenceRotation(AttitudeReference reference)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ee: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_0104: Unknown result type (might be due to invalid IL or missing references)
		//IL_0114: Unknown result type (might be due to invalid IL or missing references)
		//IL_0119: Unknown result type (might be due to invalid IL or missing references)
		//IL_0124: Unknown result type (might be due to invalid IL or missing references)
		//IL_0129: Unknown result type (might be due to invalid IL or missing references)
		//IL_012e: Unknown result type (might be due to invalid IL or missing references)
		//IL_013a: Unknown result type (might be due to invalid IL or missing references)
		//IL_013f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0144: Unknown result type (might be due to invalid IL or missing references)
		//IL_0150: Unknown result type (might be due to invalid IL or missing references)
		//IL_0155: Unknown result type (might be due to invalid IL or missing references)
		//IL_015a: Unknown result type (might be due to invalid IL or missing references)
		//IL_015b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0162: Unknown result type (might be due to invalid IL or missing references)
		//IL_0167: Unknown result type (might be due to invalid IL or missing references)
		//IL_016c: Unknown result type (might be due to invalid IL or missing references)
		//IL_016d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0172: Unknown result type (might be due to invalid IL or missing references)
		//IL_0183: Unknown result type (might be due to invalid IL or missing references)
		//IL_018e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0193: Unknown result type (might be due to invalid IL or missing references)
		//IL_0198: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01be: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01de: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0203: Unknown result type (might be due to invalid IL or missing references)
		//IL_0214: Unknown result type (might be due to invalid IL or missing references)
		//IL_0219: Unknown result type (might be due to invalid IL or missing references)
		//IL_021d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0222: Unknown result type (might be due to invalid IL or missing references)
		//IL_0227: Unknown result type (might be due to invalid IL or missing references)
		//IL_0228: Unknown result type (might be due to invalid IL or missing references)
		//IL_0229: Unknown result type (might be due to invalid IL or missing references)
		//IL_0234: Unknown result type (might be due to invalid IL or missing references)
		//IL_0239: Unknown result type (might be due to invalid IL or missing references)
		//IL_023e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0243: Unknown result type (might be due to invalid IL or missing references)
		//IL_024d: Unknown result type (might be due to invalid IL or missing references)
		//IL_024e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0253: Unknown result type (might be due to invalid IL or missing references)
		//IL_0254: Unknown result type (might be due to invalid IL or missing references)
		//IL_0259: Unknown result type (might be due to invalid IL or missing references)
		//IL_025e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0278: Unknown result type (might be due to invalid IL or missing references)
		//IL_027d: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f1: Unknown result type (might be due to invalid IL or missing references)
		//IL_02fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0301: Unknown result type (might be due to invalid IL or missing references)
		//IL_0306: Unknown result type (might be due to invalid IL or missing references)
		//IL_030b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0315: Unknown result type (might be due to invalid IL or missing references)
		//IL_0316: Unknown result type (might be due to invalid IL or missing references)
		//IL_031b: Unknown result type (might be due to invalid IL or missing references)
		//IL_031c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0321: Unknown result type (might be due to invalid IL or missing references)
		//IL_0326: Unknown result type (might be due to invalid IL or missing references)
		//IL_0348: Unknown result type (might be due to invalid IL or missing references)
		//IL_034d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0352: Unknown result type (might be due to invalid IL or missing references)
		//IL_0353: Unknown result type (might be due to invalid IL or missing references)
		//IL_0354: Unknown result type (might be due to invalid IL or missing references)
		//IL_035f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0364: Unknown result type (might be due to invalid IL or missing references)
		//IL_0369: Unknown result type (might be due to invalid IL or missing references)
		//IL_036e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0378: Unknown result type (might be due to invalid IL or missing references)
		//IL_0379: Unknown result type (might be due to invalid IL or missing references)
		//IL_037e: Unknown result type (might be due to invalid IL or missing references)
		//IL_037f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0384: Unknown result type (might be due to invalid IL or missing references)
		//IL_0389: Unknown result type (might be due to invalid IL or missing references)
		//IL_038a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0391: Unknown result type (might be due to invalid IL or missing references)
		//IL_0396: Unknown result type (might be due to invalid IL or missing references)
		//IL_039b: Unknown result type (might be due to invalid IL or missing references)
		//IL_039c: Unknown result type (might be due to invalid IL or missing references)
		//IL_03a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_044c: Unknown result type (might be due to invalid IL or missing references)
		//IL_045c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0461: Unknown result type (might be due to invalid IL or missing references)
		//IL_046c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0471: Unknown result type (might be due to invalid IL or missing references)
		//IL_0476: Unknown result type (might be due to invalid IL or missing references)
		//IL_0477: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b7: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_02be: Unknown result type (might be due to invalid IL or missing references)
		//IL_0291: Unknown result type (might be due to invalid IL or missing references)
		//IL_0293: Unknown result type (might be due to invalid IL or missing references)
		//IL_029a: Unknown result type (might be due to invalid IL or missing references)
		//IL_029f: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_03e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_03f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_03fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_03ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0404: Unknown result type (might be due to invalid IL or missing references)
		//IL_0405: Unknown result type (might be due to invalid IL or missing references)
		//IL_040a: Unknown result type (might be due to invalid IL or missing references)
		//IL_040e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0413: Unknown result type (might be due to invalid IL or missing references)
		//IL_0417: Unknown result type (might be due to invalid IL or missing references)
		//IL_041c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0421: Unknown result type (might be due to invalid IL or missing references)
		//IL_0422: Unknown result type (might be due to invalid IL or missing references)
		//IL_0427: Unknown result type (might be due to invalid IL or missing references)
		//IL_042c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0431: Unknown result type (might be due to invalid IL or missing references)
		//IL_0432: Unknown result type (might be due to invalid IL or missing references)
		//IL_0433: Unknown result type (might be due to invalid IL or missing references)
		//IL_0438: Unknown result type (might be due to invalid IL or missing references)
		//IL_0439: Unknown result type (might be due to invalid IL or missing references)
		//IL_043e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0443: Unknown result type (might be due to invalid IL or missing references)
		QuaternionD result = QuaternionD.identity;
		if (Core.Target.Target == null && (reference == AttitudeReference.TARGET || reference == AttitudeReference.TARGET_ORIENTATION || reference == AttitudeReference.RELATIVE_VELOCITY))
		{
			attitudeDeactivate();
			return result;
		}
		if ((reference == AttitudeReference.MANEUVER_NODE || reference == AttitudeReference.MANEUVER_NODE_COT) && base.Vessel.patchedConicSolver.maneuverNodes.Count == 0)
		{
			attitudeDeactivate();
			return result;
		}
		Vector3d fromDirection = base.VesselState.ThrustForward;
		if (Core.Thrust.DifferentialThrottle)
		{
			fromDirection = base.VesselState.Forward;
		}
		Vector3d val2;
		switch (reference)
		{
		case AttitudeReference.INERTIAL_COT:
			result = MathExtensions.FromToRotation(fromDirection, base.VesselState.Forward);
			break;
		case AttitudeReference.ORBIT:
			result = QuaternionD.LookRotation(((Vector3d)(ref base.VesselState.OrbitalVelocity)).normalized, base.VesselState.Up);
			break;
		case AttitudeReference.ORBIT_HORIZONTAL:
			result = QuaternionD.LookRotation(Vector3d.Exclude(base.VesselState.Up, ((Vector3d)(ref base.VesselState.OrbitalVelocity)).normalized), base.VesselState.Up);
			break;
		case AttitudeReference.SURFACE_NORTH:
			result = QuaternionD.op_Implicit(base.VesselState.RotationSurface);
			break;
		case AttitudeReference.SURFACE_NORTH_COT:
			result = QuaternionD.op_Implicit(base.VesselState.RotationSurface);
			result = MathExtensions.FromToRotation(fromDirection, base.VesselState.Forward) * result;
			break;
		case AttitudeReference.SURFACE_VELOCITY:
			result = QuaternionD.LookRotation(((Vector3d)(ref base.VesselState.SurfaceVelocity)).normalized, base.VesselState.Up);
			break;
		case AttitudeReference.TARGET:
		{
			Vector3 val4 = Core.Target.Position - base.Vessel.GetTransform().position;
			Vector3 val3 = ((Vector3)(ref val4)).normalized;
			Vector3 val = Vector3d.op_Implicit(Vector3d.Cross(Vector3d.op_Implicit(val3), base.VesselState.NormalPlus));
			Vector3.OrthoNormalize(ref val3, ref val);
			result = QuaternionD.LookRotation(Vector3d.op_Implicit(val3), Vector3d.op_Implicit(val));
			break;
		}
		case AttitudeReference.RELATIVE_VELOCITY:
		{
			val2 = Core.Target.RelativeVelocity;
			Vector3 val3 = Vector3d.op_Implicit(((Vector3d)(ref val2)).normalized);
			Vector3 val = Vector3d.op_Implicit(Vector3d.Cross(Vector3d.op_Implicit(val3), base.VesselState.NormalPlus));
			Vector3.OrthoNormalize(ref val3, ref val);
			result = QuaternionD.LookRotation(Vector3d.op_Implicit(val3), Vector3d.op_Implicit(val));
			break;
		}
		case AttitudeReference.TARGET_ORIENTATION:
		{
			Transform transform = Core.Target.Transform;
			Vector3 up = transform.up;
			result = (Core.Target.CanAlign ? QuaternionD.LookRotation(Vector3d.op_Implicit(transform.forward), Vector3d.op_Implicit(up)) : QuaternionD.LookRotation(Vector3d.op_Implicit(up), Vector3d.op_Implicit(transform.right)));
			break;
		}
		case AttitudeReference.MANEUVER_NODE:
		{
			Vector3 val3 = Vector3d.op_Implicit(base.Vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(base.Orbit));
			Vector3 val = Vector3d.op_Implicit(Vector3d.Cross(Vector3d.op_Implicit(val3), base.VesselState.NormalPlus));
			Vector3.OrthoNormalize(ref val3, ref val);
			result = QuaternionD.LookRotation(Vector3d.op_Implicit(val3), Vector3d.op_Implicit(val));
			break;
		}
		case AttitudeReference.MANEUVER_NODE_COT:
		{
			Vector3 val3 = Vector3d.op_Implicit(base.Vessel.patchedConicSolver.maneuverNodes[0].GetBurnVector(base.Orbit));
			Vector3 val = Vector3d.op_Implicit(Vector3d.Cross(Vector3d.op_Implicit(val3), base.VesselState.NormalPlus));
			Vector3.OrthoNormalize(ref val3, ref val);
			result = QuaternionD.LookRotation(Vector3d.op_Implicit(val3), Vector3d.op_Implicit(val));
			result = MathExtensions.FromToRotation(fromDirection, base.VesselState.Forward) * result;
			break;
		}
		case AttitudeReference.SUN:
		{
			Orbit obj = (((Object)(object)base.Vessel.mainBody == (Object)(object)Planetarium.fetch.Sun) ? base.Vessel.orbit : base.Orbit.TopParentOrbit());
			Vector3 val = Vector3d.op_Implicit(base.VesselState.CoM - ((Component)Planetarium.fetch.Sun).transform.position);
			val2 = obj.GetOrbitNormal();
			val2 = ((Vector3d)(ref val2)).xzy;
			Vector3 val3 = Vector3d.op_Implicit(Vector3d.Cross(-((Vector3d)(ref val2)).normalized, Vector3d.op_Implicit(val)));
			result = QuaternionD.LookRotation(Vector3d.op_Implicit(val3), Vector3d.op_Implicit(val));
			break;
		}
		case AttitudeReference.SURFACE_HORIZONTAL:
			result = QuaternionD.LookRotation(Vector3d.Exclude(base.VesselState.Up, ((Vector3d)(ref base.VesselState.SurfaceVelocity)).normalized), base.VesselState.Up);
			break;
		}
		return result;
	}

	private Vector3d attitudeWorldToReference(Vector3d vector, AttitudeReference reference)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		return QuaternionD.Inverse(attitudeGetReferenceRotation(reference)) * vector;
	}

	private Vector3d attitudeReferenceToWorld(Vector3d vector, AttitudeReference reference)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		return attitudeGetReferenceRotation(reference) * vector;
	}

	public void attitudeTo(QuaternionD attitude, AttitudeReference reference, object controller, bool AxisCtrlPitch = true, bool AxisCtrlYaw = true, bool AxisCtrlRoll = true)
	{
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		Users.Add(controller);
		attitudeReference = reference;
		attitudeTarget = attitude;
		SetOmegaTarget();
		SetAxisControl(AxisCtrlPitch, AxisCtrlYaw, AxisCtrlRoll);
	}

	public void attitudeTo(Vector3d direction, AttitudeReference reference, object controller, bool killRollRotation = false)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0051: Unknown result type (might be due to invalid IL or missing references)
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_006c: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		Vector3 val = Vector3d.op_Implicit(direction);
		Vector3 val2 = (base.Enabled ? Vector3d.op_Implicit(attitudeWorldToReference(attitudeReferenceToWorld(attitudeTarget * Vector3d.up, reference), reference)) : Vector3d.op_Implicit(attitudeWorldToReference(Vector3d.op_Implicit(-base.Vessel.GetTransform().forward), reference)));
		Vector3.OrthoNormalize(ref val, ref val2);
		attitudeTo(QuaternionD.LookRotation(Vector3d.op_Implicit(val), Vector3d.op_Implicit(val2)), reference, controller, AxisCtrlPitch: true, AxisCtrlYaw: true, killRollRotation);
		if (killRollRotation)
		{
			SetOmegaTarget(double.NaN, double.NaN, 0.0);
		}
	}

	public void attitudeTo(double heading, double pitch, double roll, object controller, bool AxisCtrlPitch = true, bool AxisCtrlYaw = true, bool AxisCtrlRoll = true, bool fixCOT = false)
	{
		//IL_0003: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_003d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		QuaternionD attitude = QuaternionD.AngleAxis((double)(float)heading, Vector3d.op_Implicit(Vector3.up)) * QuaternionD.AngleAxis((double)(0f - (float)pitch), Vector3d.op_Implicit(Vector3.right)) * QuaternionD.AngleAxis((double)(0f - (float)roll), Vector3d.op_Implicit(Vector3.forward));
		AttitudeReference reference = (fixCOT ? AttitudeReference.SURFACE_NORTH_COT : AttitudeReference.SURFACE_NORTH);
		attitudeTo(attitude, reference, controller, AxisCtrlPitch, AxisCtrlYaw, AxisCtrlRoll);
	}

	public void attitudeDeactivate()
	{
		Users.Clear();
		attitudeChanged = true;
	}

	public double attitudeAngleFromTarget()
	{
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		if (!base.Enabled)
		{
			return 0.0;
		}
		return Math.Abs(Vector3d.Angle(attitudeGetReferenceRotation(attitudeReference) * attitudeTarget * Vector3d.forward, base.VesselState.Forward));
	}

	public Vector3d targetAttitude()
	{
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		if (base.Enabled)
		{
			return attitudeGetReferenceRotation(attitudeReference) * attitudeTarget * Vector3d.forward;
		}
		return Vector3d.zero;
	}

	public override void OnFixedUpdate()
	{
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00de: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f7: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0065: Unknown result type (might be due to invalid IL or missing references)
		//IL_007b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0089: Unknown result type (might be due to invalid IL or missing references)
		//IL_008e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		steeringError.Value = (attitudeError = attitudeAngleFromTarget());
		if (!useSAS)
		{
			torque = base.VesselState.TorqueAvailable;
			if (Core.Thrust.DifferentialThrottle && Core.Thrust.DifferentialThrottleSuccess == MechJebModuleThrustController.DifferentialThrottleStatus.SUCCESS)
			{
				torque += base.VesselState.TorqueDifferentialThrottle * (double)base.Vessel.ctrlState.mainThrottle / 2.0;
			}
			inertia = 0.5 * Vector3d.Scale(base.VesselState.AngularMomentum.Sign(), Vector3d.Scale(Vector3d.Scale(base.VesselState.AngularMomentum, base.VesselState.AngularMomentum), Vector3d.Scale(torque, base.VesselState.MoI).InvertNoNaN()));
			Controller.OnFixedUpdate();
		}
	}

	public override void OnUpdate()
	{
		if (attitudeChanged)
		{
			if (attitudeReference != 0 && attitudeReference != AttitudeReference.INERTIAL_COT)
			{
				attitudeKILLROT = false;
			}
			if (!(Controller is BetterController) && !(Controller is LQRController))
			{
				Controller.Reset();
			}
			attitudeChanged = false;
		}
		Controller.OnUpdate();
	}

	public override void Drive(FlightCtrlState s)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_019e: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0060: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_020c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0211: Unknown result type (might be due to invalid IL or missing references)
		//IL_0216: Unknown result type (might be due to invalid IL or missing references)
		//IL_021b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0156: Unknown result type (might be due to invalid IL or missing references)
		//IL_015b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0160: Unknown result type (might be due to invalid IL or missing references)
		//IL_0165: Unknown result type (might be due to invalid IL or missing references)
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0113: Unknown result type (might be due to invalid IL or missing references)
		//IL_0118: Unknown result type (might be due to invalid IL or missing references)
		//IL_011d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0122: Unknown result type (might be due to invalid IL or missing references)
		//IL_0127: Unknown result type (might be due to invalid IL or missing references)
		//IL_0134: Unknown result type (might be due to invalid IL or missing references)
		//IL_0139: Unknown result type (might be due to invalid IL or missing references)
		//IL_0180: Unknown result type (might be due to invalid IL or missing references)
		//IL_0185: Unknown result type (might be due to invalid IL or missing references)
		RequestedAttitude = attitudeGetReferenceRotation(attitudeReference) * attitudeTarget;
		if (useSAS)
		{
			RequestedAttitude = attitudeGetReferenceRotation(attitudeReference) * attitudeTarget * MathExtensions.Euler(90.0, 0.0, 0.0);
			if (!base.Part.vessel.ActionGroups[(KSPActionGroup)16])
			{
				base.Part.vessel.ActionGroups.SetGroup((KSPActionGroup)16, true);
				base.Part.vessel.Autopilot.SAS.SetTargetOrientation(Vector3d.op_Implicit(RequestedAttitude * Vector3d.op_Implicit(Vector3.up)), false);
				lastSAS = RequestedAttitude;
			}
			else if (QuaternionD.Angle(lastSAS, RequestedAttitude) > 10.0)
			{
				base.Part.vessel.Autopilot.SAS.SetTargetOrientation(Vector3d.op_Implicit(RequestedAttitude * Vector3d.op_Implicit(Vector3.up)), false);
				lastSAS = RequestedAttitude;
			}
			else
			{
				base.Part.vessel.Autopilot.SAS.SetTargetOrientation(Vector3d.op_Implicit(RequestedAttitude * Vector3d.op_Implicit(Vector3.up)), true);
			}
			Core.Thrust.DifferentialThrottleDemandedTorque = Vector3d.zero;
		}
		else
		{
			Controller.DrivePre(s, out var act, out var deltaEuler);
			((Vector3d)(ref act)).Scale(ActuationControl);
			SetFlightCtrlState(act, deltaEuler, s, 1f);
			((Vector3d)(ref act))._002Ector((double)s.pitch, (double)s.roll, (double)s.yaw);
			if (Core.Thrust.DifferentialThrottleSuccess == MechJebModuleThrustController.DifferentialThrottleStatus.SUCCESS)
			{
				Core.Thrust.DifferentialThrottleDemandedTorque = -Vector3d.Scale(act, base.VesselState.TorqueDifferentialThrottle * (double)base.Vessel.ctrlState.mainThrottle);
			}
		}
	}

	private void SetFlightCtrlState(Vector3d act, Vector3d deltaEuler, FlightCtrlState s, float drive_limit)
	{
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_011a: Unknown result type (might be due to invalid IL or missing references)
		//IL_018e: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_0128: Unknown result type (might be due to invalid IL or missing references)
		//IL_0260: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0144: Unknown result type (might be due to invalid IL or missing references)
		//IL_0271: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0168: Unknown result type (might be due to invalid IL or missing references)
		//IL_0152: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0282: Unknown result type (might be due to invalid IL or missing references)
		//IL_0176: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b7: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c8: Unknown result type (might be due to invalid IL or missing references)
		bool flag = !Mathfx.Approx(s.pitch, s.pitchTrim, 0.1f);
		bool flag2 = !Mathfx.Approx(s.yaw, s.yawTrim, 0.1f);
		bool flag3 = !Mathfx.Approx(s.roll, s.rollTrim, 0.1f);
		if ((int)TimeWarp.WarpMode != 0 || TimeWarp.CurrentRateIndex == 0)
		{
			base.Part.vessel.ActionGroups.SetGroup((KSPActionGroup)16, false);
		}
		if (attitudeKILLROT && ((Object)(object)lastReferencePart != (Object)(object)base.Vessel.GetReferenceTransformPart() || flag || flag2 || flag3))
		{
			attitudeTo(QuaternionD.LookRotation(Vector3d.op_Implicit(base.Vessel.GetTransform().up), Vector3d.op_Implicit(-base.Vessel.GetTransform().forward)), AttitudeReference.INERTIAL, null);
			lastReferencePart = base.Vessel.GetReferenceTransformPart();
		}
		if (flag)
		{
			Controller.Reset(0);
		}
		if (flag3)
		{
			Controller.Reset(1);
		}
		if (flag2)
		{
			Controller.Reset(2);
		}
		if (!flag3 && !double.IsNaN(act.y))
		{
			s.roll = Mathf.Clamp((float)act.y, 0f - drive_limit, drive_limit);
		}
		if (!flag && !flag2)
		{
			if (!double.IsNaN(act.x))
			{
				s.pitch = Mathf.Clamp((float)act.x, 0f - drive_limit, drive_limit);
			}
			if (!double.IsNaN(act.z))
			{
				s.yaw = Mathf.Clamp((float)act.z, 0f - drive_limit, drive_limit);
			}
		}
		Vector3d val = default(Vector3d);
		val.x = Math.Abs(deltaEuler.x);
		val.y = Math.Abs(deltaEuler.y);
		val.z = Math.Abs(deltaEuler.z);
		if (val.x < 0.4 && val.y < 0.4 && val.z < 0.4)
		{
			if (timeCount < 50f)
			{
				timeCount += 1f;
			}
			else if (RCS_auto && attitudeRCScontrol && Core.RCS.Users.Count == 0)
			{
				base.Part.vessel.ActionGroups.SetGroup((KSPActionGroup)8, false);
			}
		}
		else if (val.x > 1.0 || val.y > 1.0 || val.z > 1.0)
		{
			timeCount = 0f;
			if (RCS_auto && (val.x > 3.0 || val.y > 3.0 || val.z > 3.0) && Core.Thrust.Limiter != MechJebModuleThrustController.LimitMode.UNSTABLE_IGNITION && attitudeRCScontrol)
			{
				base.Part.vessel.ActionGroups.SetGroup((KSPActionGroup)8, true);
			}
		}
	}
}
