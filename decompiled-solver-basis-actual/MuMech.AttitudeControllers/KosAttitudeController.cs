using System;
using UnityEngine;

namespace MuMech.AttitudeControllers;

internal class KosAttitudeController : BaseAttitudeController
{
	[Persistent(pass = 4)]
	public readonly EditableDouble MaxStoppingTime = new EditableDouble(2.0);

	[Persistent(pass = 4)]
	public readonly EditableDoubleMult RollControlRange = new EditableDoubleMult(0.0872664600610733, 0.01745329238474369);

	private readonly TorquePI _pitchPI = new TorquePI();

	private readonly TorquePI _yawPI = new TorquePI();

	private readonly TorquePI _rollPI = new TorquePI();

	private readonly KosPIDLoop _pitchRatePI = new KosPIDLoop(1.0, 0.1, 0.0, double.MaxValue, double.MinValue, extraUnwind: true);

	private readonly KosPIDLoop _yawRatePI = new KosPIDLoop(1.0, 0.1, 0.0, double.MaxValue, double.MinValue, extraUnwind: true);

	private readonly KosPIDLoop _rollRatePI = new KosPIDLoop(1.0, 0.1, 0.0, double.MaxValue, double.MinValue, extraUnwind: true);

	private Vector3d _actuation = Vector3d.zero;

	private Vector3d _targetTorque = Vector3d.zero;

	private Vector3d _omega = Vector3d.zero;

	private double _phiTotal;

	private Vector3d _phiVector = Vector3d.zero;

	private Vector3d _targetOmega = Vector3d.zero;

	private Vector3d _maxOmega = Vector3d.zero;

	private const double EPSILON = 1E-16;

	private QuaternionD _vesselRotation;

	private Vector3d _vesselForward;

	private Vector3d _vesselTop;

	private Vector3d _vesselStarboard;

	private Vector3d _targetForward;

	private Vector3d _targetTop;

	private Vector3d _controlTorque => Ac.torque;

	public KosAttitudeController(MechJebModuleAttitudeController controller)
		: base(controller)
	{
	}//IL_00fe: Unknown result type (might be due to invalid IL or missing references)
	//IL_0103: Unknown result type (might be due to invalid IL or missing references)
	//IL_0109: Unknown result type (might be due to invalid IL or missing references)
	//IL_010e: Unknown result type (might be due to invalid IL or missing references)
	//IL_0114: Unknown result type (might be due to invalid IL or missing references)
	//IL_0119: Unknown result type (might be due to invalid IL or missing references)
	//IL_011f: Unknown result type (might be due to invalid IL or missing references)
	//IL_0124: Unknown result type (might be due to invalid IL or missing references)
	//IL_012a: Unknown result type (might be due to invalid IL or missing references)
	//IL_012f: Unknown result type (might be due to invalid IL or missing references)
	//IL_0135: Unknown result type (might be due to invalid IL or missing references)
	//IL_013a: Unknown result type (might be due to invalid IL or missing references)


	public override void DrivePre(FlightCtrlState s, out Vector3d act, out Vector3d deltaEuler)
	{
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		UpdatePredictionPI();
		UpdateControl();
		deltaEuler = _phiVector * 57.295780181884766;
		act = _actuation;
	}

	private void UpdateStateVectors()
	{
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_004c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0051: Unknown result type (might be due to invalid IL or missing references)
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0062: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_006c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		//IL_0078: Unknown result type (might be due to invalid IL or missing references)
		//IL_007d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0082: Unknown result type (might be due to invalid IL or missing references)
		//IL_008e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d8: Unknown result type (might be due to invalid IL or missing references)
		_vesselRotation = QuaternionD.op_Implicit(Ac.Vessel.ReferenceTransform.rotation) * MathExtensions.Euler(-90.0, 0.0, 0.0);
		_vesselForward = _vesselRotation * Vector3d.forward;
		_vesselTop = _vesselRotation * Vector3d.up;
		_vesselStarboard = _vesselRotation * Vector3d.right;
		_targetForward = Ac.RequestedAttitude * Vector3d.forward;
		_targetTop = Ac.RequestedAttitude * Vector3d.up;
		_omega = Vector3d.op_Implicit(-Ac.Vessel.angularVelocity);
	}

	private double PhiTotal()
	{
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		UpdateStateVectors();
		double num = Vector3d.Angle(_vesselForward, _targetForward) * 0.01745329238474369;
		if (Vector3d.Angle(_vesselTop, _targetForward) > 90.0)
		{
			num *= -1.0;
		}
		return num;
	}

	private Vector3d PhiVector()
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0115: Unknown result type (might be due to invalid IL or missing references)
		//IL_011b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0121: Unknown result type (might be due to invalid IL or missing references)
		//IL_0126: Unknown result type (might be due to invalid IL or missing references)
		//IL_0156: Unknown result type (might be due to invalid IL or missing references)
		Vector3d zero = Vector3d.zero;
		((Vector3d)(ref zero))[0] = Vector3d.Angle(_vesselForward, Vector3d.Exclude(_vesselStarboard, _targetForward)) * 0.01745329238474369;
		if (Vector3d.Angle(_vesselTop, Vector3d.Exclude(_vesselStarboard, _targetForward)) > 90.0)
		{
			ref Vector3d reference = ref zero;
			((Vector3d)(ref reference))[0] = ((Vector3d)(ref reference))[0] * -1.0;
		}
		((Vector3d)(ref zero))[1] = Vector3d.Angle(_vesselTop, Vector3d.Exclude(_vesselForward, _targetTop)) * 0.01745329238474369;
		if (Vector3d.Angle(_vesselStarboard, Vector3d.Exclude(_vesselForward, _targetTop)) > 90.0)
		{
			ref Vector3d reference = ref zero;
			((Vector3d)(ref reference))[1] = ((Vector3d)(ref reference))[1] * -1.0;
		}
		((Vector3d)(ref zero))[2] = Vector3d.Angle(_vesselForward, Vector3d.Exclude(_vesselTop, _targetForward)) * 0.01745329238474369;
		if (Vector3d.Angle(_vesselStarboard, Vector3d.Exclude(_vesselTop, _targetForward)) > 90.0)
		{
			ref Vector3d reference = ref zero;
			((Vector3d)(ref reference))[2] = ((Vector3d)(ref reference))[2] * -1.0;
		}
		return zero;
	}

	private void UpdatePredictionPI()
	{
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_0181: Unknown result type (might be due to invalid IL or missing references)
		//IL_0186: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01da: Unknown result type (might be due to invalid IL or missing references)
		//IL_0229: Unknown result type (might be due to invalid IL or missing references)
		//IL_022e: Unknown result type (might be due to invalid IL or missing references)
		_phiTotal = PhiTotal();
		_phiVector = PhiVector();
		Vector3d controlTorque;
		for (int i = 0; i < 3; i++)
		{
			ref Vector3d maxOmega = ref _maxOmega;
			int num = i;
			controlTorque = _controlTorque;
			((Vector3d)(ref maxOmega))[num] = ((Vector3d)(ref controlTorque))[i] * (double)MaxStoppingTime / ((Vector3d)(ref Ac.VesselState.MoI))[i];
		}
		((Vector3d)(ref _targetOmega))[0] = _pitchRatePI.Update(0.0 - ((Vector3d)(ref _phiVector))[0], 0.0, ((Vector3d)(ref _maxOmega))[0]);
		((Vector3d)(ref _targetOmega))[1] = _rollRatePI.Update(0.0 - ((Vector3d)(ref _phiVector))[1], 0.0, ((Vector3d)(ref _maxOmega))[1]);
		((Vector3d)(ref _targetOmega))[2] = _yawRatePI.Update(0.0 - ((Vector3d)(ref _phiVector))[2], 0.0, ((Vector3d)(ref _maxOmega))[2]);
		if (Math.Abs(_phiTotal) > (double)RollControlRange)
		{
			((Vector3d)(ref _targetOmega))[1] = 0.0;
			_rollRatePI.ResetI();
		}
		ref Vector3d targetTorque = ref _targetTorque;
		TorquePI pitchPI = _pitchPI;
		double input = ((Vector3d)(ref _omega))[0];
		double setpoint = ((Vector3d)(ref _targetOmega))[0];
		double momentOfInertia = ((Vector3d)(ref Ac.VesselState.MoI))[0];
		controlTorque = _controlTorque;
		((Vector3d)(ref targetTorque))[0] = pitchPI.Update(input, setpoint, momentOfInertia, ((Vector3d)(ref controlTorque))[0]);
		ref Vector3d targetTorque2 = ref _targetTorque;
		TorquePI rollPI = _rollPI;
		double input2 = ((Vector3d)(ref _omega))[1];
		double setpoint2 = ((Vector3d)(ref _targetOmega))[1];
		double momentOfInertia2 = ((Vector3d)(ref Ac.VesselState.MoI))[1];
		controlTorque = _controlTorque;
		((Vector3d)(ref targetTorque2))[1] = rollPI.Update(input2, setpoint2, momentOfInertia2, ((Vector3d)(ref controlTorque))[1]);
		ref Vector3d targetTorque3 = ref _targetTorque;
		TorquePI yawPI = _yawPI;
		double input3 = ((Vector3d)(ref _omega))[2];
		double setpoint3 = ((Vector3d)(ref _targetOmega))[2];
		double momentOfInertia3 = ((Vector3d)(ref Ac.VesselState.MoI))[2];
		controlTorque = _controlTorque;
		((Vector3d)(ref targetTorque3))[2] = yawPI.Update(input3, setpoint3, momentOfInertia3, ((Vector3d)(ref controlTorque))[2]);
	}

	public override void Reset()
	{
		_pitchPI.ResetI();
		_yawPI.ResetI();
		_rollPI.ResetI();
		_pitchRatePI.ResetI();
		_yawRatePI.ResetI();
		_rollRatePI.ResetI();
	}

	private void UpdateControl()
	{
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		for (int i = 0; i < 3; i++)
		{
			double num = Math.Max(Math.Abs(((Vector3d)(ref _actuation))[i]), 0.005) * 2.0;
			ref Vector3d actuation = ref _actuation;
			int num2 = i;
			double num3 = ((Vector3d)(ref _targetTorque))[i];
			Vector3d controlTorque = _controlTorque;
			((Vector3d)(ref actuation))[num2] = num3 / ((Vector3d)(ref controlTorque))[i];
			if (Math.Abs(((Vector3d)(ref _actuation))[i]) < 1E-16 || double.IsNaN(((Vector3d)(ref _actuation))[i]))
			{
				((Vector3d)(ref _actuation))[i] = 0.0;
			}
			((Vector3d)(ref _actuation))[i] = Math.Max(Math.Min(((Vector3d)(ref _actuation))[i], num), 0.0 - num);
		}
	}

	public override void GUI()
	{
		//IL_00dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0127: Unknown result type (might be due to invalid IL or missing references)
		//IL_0171: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bb: Unknown result type (might be due to invalid IL or missing references)
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("MaxStoppingTime", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		MaxStoppingTime.Text = GUILayout.TextField(MaxStoppingTime.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(40f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("RollControlRange", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		RollControlRange.Text = GUILayout.TextField(RollControlRange.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(40f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Actuation", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_actuation), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("phiVector", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_phiVector), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("TargetTorque", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_targetTorque), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("ControlTorque", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_controlTorque), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
	}
}
