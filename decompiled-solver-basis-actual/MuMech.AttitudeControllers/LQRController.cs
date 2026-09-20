using System;
using JetBrains.Annotations;
using MechJebLib.Control;
using UnityEngine;

namespace MuMech.AttitudeControllers;

public class LQRController : BaseAttitudeController
{
	private readonly LQRLoop1[] _lqr = (LQRLoop1[])(object)new LQRLoop1[3]
	{
		new LQRLoop1(),
		new LQRLoop1(),
		new LQRLoop1()
	};

	private readonly DirectionTracker _directionTracker = new DirectionTracker();

	private Vector3d _actuation = Vector3d.zero;

	private Vector3d _current = Vector3d.zero;

	private Vector3d _desired = Vector3d.zero;

	private Vector3d _error = Vector3d.zero;

	private double _distance;

	private Vector3d _velocity = Vector3d.zero;

	private Vector3d _mass = Vector3d.zero;

	private Vector3d _controlTorque = Vector3d.zero;

	private Vector3d _targetTorque = Vector3d.zero;

	[UsedImplicitly]
	[Persistent(pass = 6)]
	public readonly EditableDouble SmoothTorque = new EditableDouble(0.1);

	private Vessel _vessel => Ac.Vessel;

	public LQRController(MechJebModuleAttitudeController controller)
		: base(controller)
	{
	}//IL_0009: Unknown result type (might be due to invalid IL or missing references)
	//IL_000f: Expected O, but got Unknown
	//IL_0011: Unknown result type (might be due to invalid IL or missing references)
	//IL_0017: Expected O, but got Unknown
	//IL_0019: Unknown result type (might be due to invalid IL or missing references)
	//IL_001f: Expected O, but got Unknown
	//IL_0030: Unknown result type (might be due to invalid IL or missing references)
	//IL_0035: Unknown result type (might be due to invalid IL or missing references)
	//IL_003b: Unknown result type (might be due to invalid IL or missing references)
	//IL_0040: Unknown result type (might be due to invalid IL or missing references)
	//IL_0046: Unknown result type (might be due to invalid IL or missing references)
	//IL_004b: Unknown result type (might be due to invalid IL or missing references)
	//IL_0051: Unknown result type (might be due to invalid IL or missing references)
	//IL_0056: Unknown result type (might be due to invalid IL or missing references)
	//IL_005c: Unknown result type (might be due to invalid IL or missing references)
	//IL_0061: Unknown result type (might be due to invalid IL or missing references)
	//IL_0067: Unknown result type (might be due to invalid IL or missing references)
	//IL_006c: Unknown result type (might be due to invalid IL or missing references)
	//IL_0072: Unknown result type (might be due to invalid IL or missing references)
	//IL_0077: Unknown result type (might be due to invalid IL or missing references)
	//IL_007d: Unknown result type (might be due to invalid IL or missing references)
	//IL_0082: Unknown result type (might be due to invalid IL or missing references)


	private void UpdateLQR()
	{
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_0059: Unknown result type (might be due to invalid IL or missing references)
		//IL_006a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		//IL_0088: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00be: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00da: Unknown result type (might be due to invalid IL or missing references)
		//IL_00df: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_022b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0230: Unknown result type (might be due to invalid IL or missing references)
		//IL_0262: Unknown result type (might be due to invalid IL or missing references)
		//IL_0267: Unknown result type (might be due to invalid IL or missing references)
		QuaternionD currentRotation = QuaternionD.op_Implicit(((Component)_vessel.ReferenceTransform).transform.rotation) * MathExtensions.Euler(-90.0, 0.0, 0.0);
		_directionTracker.Update(currentRotation);
		_current = _directionTracker.TrackedRotation;
		(Vector3d, Vector3d, double) tuple = _directionTracker.Desired(Ac.RequestedAttitude);
		_desired = tuple.Item1;
		_error = tuple.Item2;
		_distance = tuple.Item3;
		_velocity = _vessel.angularVelocityD;
		_controlTorque = ((_controlTorque == Vector3d.zero) ? Ac.torque : (_controlTorque + (double)SmoothTorque * (Ac.torque - _controlTorque)));
		for (int i = 0; i < 3; i++)
		{
			if (((Vector3d)(ref Ac.torque))[i] == 0.0)
			{
				((Vector3d)(ref _controlTorque))[i] = 0.0;
			}
		}
		for (int num = 0; num < 3; num++)
		{
			((Vector3d)(ref _mass))[num] = (double)((Vector3)(ref _vessel.MOI))[num] / ((Vector3d)(ref _controlTorque))[num];
			_lqr[num].M = ((Vector3d)(ref _mass))[num];
			_lqr[num].Ts = Ac.VesselState.DeltaT;
			_lqr[num].Grr = 16.0;
			_lqr[num].UMin = -1.0;
			_lqr[num].UMax = 1.0;
			((Vector3d)(ref _actuation))[num] = 0.0 - _lqr[num].Update(((Vector3d)(ref _desired))[num], ((Vector3d)(ref _current))[num], ((Vector3d)(ref _velocity))[num]);
			Vector3d val = Ac.ActuationControl;
			if (((Vector3d)(ref val))[num] != 0.0 && ((Vector3d)(ref _controlTorque))[num] != 0.0)
			{
				val = Ac.AxisControl;
				if (((Vector3d)(ref val))[num] != 0.0)
				{
					goto IL_0298;
				}
			}
			((Vector3d)(ref _actuation))[num] = 0.0;
			Reset(num);
			goto IL_0298;
			IL_0298:
			if (Math.Abs(((Vector3d)(ref _actuation))[num]) < 2.220446049250313E-16 || double.IsNaN(((Vector3d)(ref _actuation))[num]))
			{
				((Vector3d)(ref _actuation))[num] = 0.0;
			}
			((Vector3d)(ref _targetTorque))[num] = ((Vector3d)(ref _controlTorque))[num] * ((Vector3d)(ref _actuation))[num];
		}
	}

	public override void DrivePre(FlightCtrlState s, out Vector3d act, out Vector3d deltaEuler)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		UpdateLQR();
		deltaEuler = -_error * 57.295780181884766;
		for (int i = 0; i < 3; i++)
		{
			if (Math.Abs(((Vector3d)(ref _actuation))[i]) < 2.220446049250313E-16 || double.IsNaN(((Vector3d)(ref _actuation))[i]))
			{
				((Vector3d)(ref _actuation))[i] = 0.0;
			}
		}
		act = _actuation;
	}

	public override void Reset()
	{
		Reset(0);
		Reset(1);
		Reset(2);
		_directionTracker.Reset();
	}

	public override void Reset(int i)
	{
		_lqr[i].Reset();
		_directionTracker.Reset(i);
	}

	public override void GUI()
	{
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_0114: Unknown result type (might be due to invalid IL or missing references)
		//IL_015e: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_0241: Unknown result type (might be due to invalid IL or missing references)
		//IL_0246: Unknown result type (might be due to invalid IL or missing references)
		//IL_0290: Unknown result type (might be due to invalid IL or missing references)
		//IL_02da: Unknown result type (might be due to invalid IL or missing references)
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Smooth Torque", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		SmoothTorque.Text = GUILayout.TextField(SmoothTorque.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Position", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_current), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Desired", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_desired), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Error", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_error), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Velocity", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_velocity), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Mass", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_mass), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Actuation", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_actuation), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("MOI", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(Vector3d.op_Implicit(_vessel.MOI)), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Control Torque", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_controlTorque), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Applied Torque", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_targetTorque), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
	}
}
