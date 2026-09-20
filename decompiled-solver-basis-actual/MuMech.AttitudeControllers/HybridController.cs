using System;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech.AttitudeControllers;

internal class HybridController : BaseAttitudeController
{
	[Persistent(pass = 4)]
	public readonly EditableDouble MaxStoppingTime = new EditableDouble(2.0);

	[Persistent(pass = 4)]
	public readonly EditableDoubleMult RollControlRange = new EditableDoubleMult(0.0872664600610733, 0.01745329238474369);

	[Persistent(pass = 4)]
	public bool UseControlRange = true;

	private readonly TorquePI _pitchPI = new TorquePI();

	private readonly TorquePI _yawPI = new TorquePI();

	private readonly TorquePI _rollPI = new TorquePI();

	private readonly KosPIDLoop _pitchRatePI = new KosPIDLoop(1.0, 0.1, 0.0, double.MaxValue, double.MinValue, extraUnwind: true);

	private readonly KosPIDLoop _yawRatePI = new KosPIDLoop(1.0, 0.1, 0.0, double.MaxValue, double.MinValue, extraUnwind: true);

	private readonly KosPIDLoop _rollRatePI = new KosPIDLoop(1.0, 0.1, 0.0, double.MaxValue, double.MinValue, extraUnwind: true);

	[Persistent(pass = 4)]
	public bool UseInertia = true;

	private Vector3d _actuation = Vector3d.zero;

	private Vector3d _targetTorque = Vector3d.zero;

	private Vector3d _omega = Vector3d.zero;

	private double _phiTotal;

	private Vector3d _phiVector = Vector3d.zero;

	private Vector3d _targetOmega = Vector3d.zero;

	private Vector3d _maxOmega = Vector3d.zero;

	private const double EPSILON = 1E-16;

	private Vector3d _controlTorque => Ac.torque;

	public HybridController(MechJebModuleAttitudeController controller)
		: base(controller)
	{
	}//IL_010c: Unknown result type (might be due to invalid IL or missing references)
	//IL_0111: Unknown result type (might be due to invalid IL or missing references)
	//IL_0117: Unknown result type (might be due to invalid IL or missing references)
	//IL_011c: Unknown result type (might be due to invalid IL or missing references)
	//IL_0122: Unknown result type (might be due to invalid IL or missing references)
	//IL_0127: Unknown result type (might be due to invalid IL or missing references)
	//IL_012d: Unknown result type (might be due to invalid IL or missing references)
	//IL_0132: Unknown result type (might be due to invalid IL or missing references)
	//IL_0138: Unknown result type (might be due to invalid IL or missing references)
	//IL_013d: Unknown result type (might be due to invalid IL or missing references)
	//IL_0143: Unknown result type (might be due to invalid IL or missing references)
	//IL_0148: Unknown result type (might be due to invalid IL or missing references)


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

	private void UpdatePhi()
	{
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_0059: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_012e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0155: Unknown result type (might be due to invalid IL or missing references)
		//IL_0157: Unknown result type (might be due to invalid IL or missing references)
		//IL_0140: Unknown result type (might be due to invalid IL or missing references)
		//IL_0148: Unknown result type (might be due to invalid IL or missing references)
		//IL_014d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0152: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = MathExtensions.EulerAngles(QuaternionD.Inverse(QuaternionD.op_Implicit(((Component)Ac.Vessel.ReferenceTransform).transform.rotation) * MathExtensions.Euler(-90.0, 0.0, 0.0)) * Ac.RequestedAttitude);
		double num = ((Vector3d)(ref val))[0] * (Math.PI / 180.0);
		double num2 = ((Vector3d)(ref val))[1] * (Math.PI / 180.0);
		double num3 = ((Vector3d)(ref val))[2] * (Math.PI / 180.0);
		_phiTotal = Math.Acos(Statics.Clamp(Math.Cos(num) * Math.Cos(num2), -1.0, 1.0));
		Vector3d val2 = default(Vector3d);
		((Vector3d)(ref val2))._002Ector(Math.Sin(num), Math.Cos(num) * Math.Sin(0.0 - num2), 0.0);
		val2 = ((Vector3d)(ref val2)).normalized * _phiTotal;
		Vector3d val3 = default(Vector3d);
		((Vector3d)(ref val3))._002Ector(Statics.ClampPi(((Vector3d)(ref val2))[0]), Statics.ClampPi(num3), Statics.ClampPi(((Vector3d)(ref val2))[1]));
		((Vector3d)(ref val3)).Scale(Ac.AxisControl);
		if (UseInertia)
		{
			val3 -= Ac.inertia;
		}
		_phiVector = val3;
	}

	private void UpdatePredictionPI()
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0194: Unknown result type (might be due to invalid IL or missing references)
		//IL_0199: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_023c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0241: Unknown result type (might be due to invalid IL or missing references)
		_omega = Vector3d.op_Implicit(-Ac.Vessel.angularVelocity);
		UpdatePhi();
		Vector3d controlTorque;
		for (int i = 0; i < 3; i++)
		{
			ref Vector3d maxOmega = ref _maxOmega;
			int num = i;
			controlTorque = _controlTorque;
			((Vector3d)(ref maxOmega))[num] = ((Vector3d)(ref controlTorque))[i] * (double)MaxStoppingTime / ((Vector3d)(ref Ac.VesselState.MoI))[i];
		}
		((Vector3d)(ref _targetOmega))[0] = _pitchRatePI.Update(((Vector3d)(ref _phiVector))[0], 0.0, ((Vector3d)(ref _maxOmega))[0]);
		((Vector3d)(ref _targetOmega))[1] = _rollRatePI.Update(((Vector3d)(ref _phiVector))[1], 0.0, ((Vector3d)(ref _maxOmega))[1]);
		((Vector3d)(ref _targetOmega))[2] = _yawRatePI.Update(((Vector3d)(ref _phiVector))[2], 0.0, ((Vector3d)(ref _maxOmega))[2]);
		if (UseControlRange && Math.Abs(_phiTotal) > (double)RollControlRange)
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
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		for (int i = 0; i < 3; i++)
		{
			ref Vector3d actuation = ref _actuation;
			int num = i;
			double num2 = ((Vector3d)(ref _targetTorque))[i];
			Vector3d controlTorque = _controlTorque;
			((Vector3d)(ref actuation))[num] = num2 / ((Vector3d)(ref controlTorque))[i];
			if (Math.Abs(((Vector3d)(ref _actuation))[i]) < 1E-16 || double.IsNaN(((Vector3d)(ref _actuation))[i]))
			{
				((Vector3d)(ref _actuation))[i] = 0.0;
			}
		}
	}

	public override void GUI()
	{
		//IL_0118: Unknown result type (might be due to invalid IL or missing references)
		//IL_0167: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_024a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0299: Unknown result type (might be due to invalid IL or missing references)
		//IL_0314: Unknown result type (might be due to invalid IL or missing references)
		UseInertia = GUILayout.Toggle(UseInertia, Localizer.Format("#MechJeb_HybridController_checkbox1"), Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_HybridController_label1"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		MaxStoppingTime.Text = GUILayout.TextField(MaxStoppingTime.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(60f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		UseControlRange = GUILayout.Toggle(UseControlRange, Localizer.Format("#MechJeb_HybridController_checkbox2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		RollControlRange.Text = GUILayout.TextField(RollControlRange.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(60f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_HybridController_label2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_actuation), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_HybridController_label3"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_phiVector), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Omega", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_omega), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("MaxOmega", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_maxOmega), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_HybridController_label4"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_targetTorque), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_HybridController_label5"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_controlTorque), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_HybridController_label6"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label("|" + ((Vector3d)(ref Ac.inertia)).magnitude.ToString("F3") + "| " + MuUtils.PrettyPrint(Ac.inertia), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
	}
}
