using System;
using KSP.Localization;
using MechJebLib.Control;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech.AttitudeControllers;

internal class BetterController : BaseAttitudeController
{
	private const int SETTINGS_VERSION = 16;

	private const double POS_KP_DEFAULT = 2.03;

	private const double POS_TI_DEFAULT = 1.97;

	private const double POS_TD_DEFAULT = 0.0;

	private const double POS_N_DEFAULT = 1.0;

	private const double POS_B_DEFAULT = 1.0;

	private const double POS_C_DEFAULT = 1.0;

	private const double POS_DEADBAND_DEFAULT = 0.0;

	private const bool POS_CLEGG_DEFAULT = false;

	private const double POS_SMOOTH_IN_DEFAULT = 1.0;

	private const double POS_SMOOTH_OUT_DEFAULT = 1.0;

	private const double VEL_KP_DEFAULT = 7.98;

	private const double VEL_TI_DEFAULT = 0.0;

	private const double VEL_TD_DEFAULT = 0.0;

	private const double VEL_N_DEFAULT = 1.0;

	private const double VEL_B_DEFAULT = 1.0;

	private const double VEL_C_DEFAULT = 1.0;

	private const double VEL_DEADBAND_DEFAULT = 0.0;

	private const bool VEL_CLEGG_DEFAULT = false;

	private const double VEL_SMOOTH_IN_DEFAULT = 1.0;

	private const double VEL_SMOOTH_OUT_DEFAULT = 1.0;

	private const double MAX_STOPPING_TIME_DEFAULT = 2.0;

	private const double MIN_FLIP_TIME_DEFAULT = 120.0;

	private const double ROLL_CONTROL_RANGE_DEFAULT = 5.0;

	private const double SMOOTH_TORQUE_DEFAULT = 0.1;

	private const double SOFTEN_DEFAULT = 0.5;

	private readonly PIDLoop2[] _velPID = (PIDLoop2[])(object)new PIDLoop2[3]
	{
		new PIDLoop2(),
		new PIDLoop2(),
		new PIDLoop2()
	};

	private readonly PIDLoop2[] _posPID = (PIDLoop2[])(object)new PIDLoop2[3]
	{
		new PIDLoop2(),
		new PIDLoop2(),
		new PIDLoop2()
	};

	private readonly DirectionTracker _directionTracker = new DirectionTracker();

	[Persistent(pass = 6)]
	public readonly EditableDouble MaxStoppingTime = new EditableDouble(2.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble MinFlipTime = new EditableDouble(120.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble PosDeadband = new EditableDouble(0.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble PosKp = new EditableDouble(2.03);

	[Persistent(pass = 6)]
	public readonly EditableDouble PosTi = new EditableDouble(1.97);

	[Persistent(pass = 6)]
	public readonly EditableDouble PosTd = new EditableDouble(0.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble PosN = new EditableDouble(1.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble PosB = new EditableDouble(1.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble PosC = new EditableDouble(1.0);

	[Persistent(pass = 6)]
	public bool PosClegg;

	[Persistent(pass = 6)]
	public readonly EditableDouble PosSmoothIn = new EditableDouble(1.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble PosSmoothOut = new EditableDouble(1.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble RollControlRange = new EditableDouble(5.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble VelB = new EditableDouble(1.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble VelC = new EditableDouble(1.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble VelDeadband = new EditableDouble(0.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble VelKp = new EditableDouble(7.98);

	[Persistent(pass = 6)]
	public readonly EditableDouble VelN = new EditableDouble(1.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble VelSmoothIn = new EditableDouble(1.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble VelSmoothOut = new EditableDouble(1.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble VelTd = new EditableDouble(0.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble VelTi = new EditableDouble(0.0);

	[Persistent(pass = 6)]
	public readonly EditableDouble Soften = new EditableDouble(0.5);

	private Vector3d _actuation = Vector3d.zero;

	private Vector3d _error = Vector3d.zero;

	private Vector3d _current = Vector3d.zero;

	private Vector3d _desired = Vector3d.zero;

	private double _distance;

	private Vector3d _maxAlpha = Vector3d.zero;

	private Vector3d _targetOmega = Vector3d.zero;

	private Vector3d _targetAlpha = Vector3d.zero;

	private Vector3d _targetTorque = Vector3d.zero;

	private Vector3d _controlTorque = Vector3d.zero;

	[Persistent(pass = 6)]
	public readonly EditableDouble SmoothTorque = new EditableDouble(0.1);

	[Persistent(pass = 6)]
	public bool UseControlRange = true;

	[Persistent(pass = 6)]
	public bool UseFlipTime = true;

	[Persistent(pass = 6)]
	public bool UseStoppingTime = true;

	[Persistent(pass = 6)]
	public bool VelClegg;

	[Persistent(pass = 6)]
	public int Version = -1;

	private Vessel _vessel => Ac.Vessel;

	public BetterController(MechJebModuleAttitudeController controller)
		: base(controller)
	{
	}//IL_0009: Unknown result type (might be due to invalid IL or missing references)
	//IL_000f: Expected O, but got Unknown
	//IL_0011: Unknown result type (might be due to invalid IL or missing references)
	//IL_0017: Expected O, but got Unknown
	//IL_0019: Unknown result type (might be due to invalid IL or missing references)
	//IL_001f: Expected O, but got Unknown
	//IL_002d: Unknown result type (might be due to invalid IL or missing references)
	//IL_0033: Expected O, but got Unknown
	//IL_0035: Unknown result type (might be due to invalid IL or missing references)
	//IL_003b: Expected O, but got Unknown
	//IL_003d: Unknown result type (might be due to invalid IL or missing references)
	//IL_0043: Expected O, but got Unknown
	//IL_020c: Unknown result type (might be due to invalid IL or missing references)
	//IL_0211: Unknown result type (might be due to invalid IL or missing references)
	//IL_0217: Unknown result type (might be due to invalid IL or missing references)
	//IL_021c: Unknown result type (might be due to invalid IL or missing references)
	//IL_0222: Unknown result type (might be due to invalid IL or missing references)
	//IL_0227: Unknown result type (might be due to invalid IL or missing references)
	//IL_022d: Unknown result type (might be due to invalid IL or missing references)
	//IL_0232: Unknown result type (might be due to invalid IL or missing references)
	//IL_0238: Unknown result type (might be due to invalid IL or missing references)
	//IL_023d: Unknown result type (might be due to invalid IL or missing references)
	//IL_0243: Unknown result type (might be due to invalid IL or missing references)
	//IL_0248: Unknown result type (might be due to invalid IL or missing references)
	//IL_024e: Unknown result type (might be due to invalid IL or missing references)
	//IL_0253: Unknown result type (might be due to invalid IL or missing references)
	//IL_0259: Unknown result type (might be due to invalid IL or missing references)
	//IL_025e: Unknown result type (might be due to invalid IL or missing references)
	//IL_0264: Unknown result type (might be due to invalid IL or missing references)
	//IL_0269: Unknown result type (might be due to invalid IL or missing references)


	private void Defaults()
	{
		PosKp.Val = 2.03;
		PosTi.Val = 1.97;
		PosTd.Val = 0.0;
		PosN.Val = 1.0;
		PosB.Val = 1.0;
		PosC.Val = 1.0;
		PosDeadband.Val = 0.0;
		PosSmoothOut.Val = 1.0;
		PosSmoothIn.Val = 1.0;
		PosClegg = false;
		VelKp.Val = 7.98;
		VelTi.Val = 0.0;
		VelTd.Val = 0.0;
		VelN.Val = 1.0;
		VelB.Val = 1.0;
		VelC.Val = 1.0;
		VelDeadband.Val = 0.0;
		VelSmoothIn.Val = 1.0;
		VelSmoothOut.Val = 1.0;
		VelClegg = false;
		MaxStoppingTime.Val = 2.0;
		MinFlipTime.Val = 120.0;
		RollControlRange.Val = 5.0;
		UseControlRange = true;
		UseFlipTime = true;
		UseStoppingTime = true;
		SmoothTorque.Val = 0.1;
		Soften.Val = 0.5;
		Version = 16;
	}

	public override void OnModuleEnabled()
	{
		if (Version < 16)
		{
			Defaults();
		}
		Reset();
	}

	public override void DrivePre(FlightCtrlState s, out Vector3d act, out Vector3d deltaEuler)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		UpdatePredictionPI();
		deltaEuler = _error * 57.295780181884766;
		for (int i = 0; i < 3; i++)
		{
			if (Math.Abs(((Vector3d)(ref _actuation))[i]) < 2.220446049250313E-16 || double.IsNaN(((Vector3d)(ref _actuation))[i]))
			{
				((Vector3d)(ref _actuation))[i] = 0.0;
			}
		}
		act = _actuation;
	}

	private void UpdatePredictionPI()
	{
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0070: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00de: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_06db: Unknown result type (might be due to invalid IL or missing references)
		//IL_06e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_0714: Unknown result type (might be due to invalid IL or missing references)
		//IL_0719: Unknown result type (might be due to invalid IL or missing references)
		QuaternionD currentRotation = QuaternionD.op_Implicit(((Component)_vessel.ReferenceTransform).transform.rotation) * MathExtensions.Euler(-90.0, 0.0, 0.0);
		_current = _directionTracker.Update(currentRotation);
		(Vector3d, Vector3d, double) tuple = _directionTracker.Desired(Ac.RequestedAttitude);
		_desired = tuple.Item1;
		_error = tuple.Item2;
		_distance = tuple.Item3;
		_controlTorque = ((_controlTorque == Vector3d.zero) ? Ac.torque : (_controlTorque + (double)SmoothTorque * (Ac.torque - _controlTorque)));
		for (int i = 0; i < 3; i++)
		{
			if (((Vector3d)(ref Ac.torque))[i] == 0.0)
			{
				((Vector3d)(ref _controlTorque))[i] = 0.0;
			}
		}
		double num = Ac.VesselState.DeltaT / 0.02;
		for (int num2 = 0; num2 < 3; num2++)
		{
			((Vector3d)(ref _maxAlpha))[num2] = ((Vector3d)(ref _controlTorque))[num2] / (double)((Vector3)(ref _vessel.MOI))[num2];
			if (((Vector3d)(ref _maxAlpha))[num2] == 0.0)
			{
				((Vector3d)(ref _maxAlpha))[num2] = 1.0;
			}
			Vector3d val = Ac.OmegaTarget;
			if (((Vector3d)(ref val))[num2].IsFinite())
			{
				ref Vector3d targetOmega = ref _targetOmega;
				int num3 = num2;
				val = Ac.OmegaTarget;
				((Vector3d)(ref targetOmega))[num3] = ((Vector3d)(ref val))[num2];
			}
			else
			{
				double num4 = Statics.Clamp01((double)Soften);
				double num5 = (double)PosKp / num;
				double num6 = num4 * num4 * ((Vector3d)(ref _maxAlpha))[num2] / (2.0 * num5 * num5);
				double num7 = double.PositiveInfinity;
				if (UseStoppingTime)
				{
					num7 = ((Vector3d)(ref _maxAlpha))[num2] * (double)MaxStoppingTime;
					if (UseFlipTime)
					{
						num7 = Math.Max(num7, Math.PI / MinFlipTime.Val);
					}
				}
				if (Math.Abs(((Vector3d)(ref _error))[num2]) <= 2.0 * num6)
				{
					_posPID[num2].Kp = num5;
					_posPID[num2].Ti = PosTi;
					_posPID[num2].Td = PosTd;
					_posPID[num2].N = PosN.Val;
					_posPID[num2].B = PosB.Val;
					_posPID[num2].C = PosC.Val;
					_posPID[num2].Ts = Ac.VesselState.DeltaT;
					_posPID[num2].SmoothIn = Statics.Clamp01((double)PosSmoothIn);
					_posPID[num2].SmoothOut = Statics.Clamp01((double)PosSmoothOut);
					_posPID[num2].MinOutput = 0.0 - num7;
					_posPID[num2].MaxOutput = num7;
					_posPID[num2].IntegralDeadband = (double)PosDeadband * num7;
					_posPID[num2].Clegg = PosClegg;
					((Vector3d)(ref _targetOmega))[num2] = _posPID[num2].Update(((Vector3d)(ref _desired))[num2], ((Vector3d)(ref _current))[num2]);
				}
				else
				{
					_posPID[num2].Reset();
					((Vector3d)(ref _targetOmega))[num2] = num4 * Math.Sqrt(2.0 * ((Vector3d)(ref _maxAlpha))[num2] * (Math.Abs(((Vector3d)(ref _error))[num2]) - num6)) * (double)Math.Sign(((Vector3d)(ref _error))[num2]);
					((Vector3d)(ref _targetOmega))[num2] = Statics.Clamp(((Vector3d)(ref _targetOmega))[num2], 0.0 - num7, num7);
				}
				if (UseControlRange && _distance * 57.295780181884766 > (double)RollControlRange)
				{
					((Vector3d)(ref _targetOmega))[1] = 0.0;
					_posPID[1].Reset();
				}
			}
			_velPID[num2].Kp = VelKp;
			_velPID[num2].Ti = VelTi;
			_velPID[num2].Td = VelTd;
			_velPID[num2].N = VelN;
			_velPID[num2].B = VelB;
			_velPID[num2].C = VelC;
			_velPID[num2].Ts = Ac.VesselState.DeltaT;
			_velPID[num2].SmoothIn = Statics.Clamp01((double)VelSmoothIn);
			_velPID[num2].SmoothOut = Statics.Clamp01((double)VelSmoothOut);
			_velPID[num2].MinOutput = 0.0 - ((Vector3d)(ref _maxAlpha))[num2];
			_velPID[num2].MaxOutput = ((Vector3d)(ref _maxAlpha))[num2];
			_velPID[num2].IntegralDeadband = (double)VelDeadband * ((Vector3d)(ref _maxAlpha))[num2];
			_velPID[num2].Clegg = VelClegg;
			((Vector3d)(ref _targetAlpha))[num2] = _velPID[num2].Update(((Vector3d)(ref _targetOmega))[num2], ((Vector3d)(ref _vessel.angularVelocityD))[num2]);
			((Vector3d)(ref _targetTorque))[num2] = (double)((Vector3)(ref _vessel.MOI))[num2] * ((Vector3d)(ref _targetAlpha))[num2];
			((Vector3d)(ref _actuation))[num2] = (0.0 - ((Vector3d)(ref _targetTorque))[num2]) / ((Vector3d)(ref _controlTorque))[num2];
			val = Ac.ActuationControl;
			if (((Vector3d)(ref val))[num2] != 0.0 && ((Vector3d)(ref _controlTorque))[num2] != 0.0)
			{
				val = Ac.AxisControl;
				if (((Vector3d)(ref val))[num2] != 0.0)
				{
					goto IL_074d;
				}
			}
			((Vector3d)(ref _actuation))[num2] = 0.0;
			Reset(num2);
			goto IL_074d;
			IL_074d:
			if (Math.Abs(((Vector3d)(ref _actuation))[num2]) < 2.220446049250313E-16 || double.IsNaN(((Vector3d)(ref _actuation))[num2]))
			{
				((Vector3d)(ref _actuation))[num2] = 0.0;
			}
		}
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
		_velPID[i].Reset();
		_posPID[i].Reset();
		_directionTracker.Reset(i);
	}

	public override void GUI()
	{
		//IL_0856: Unknown result type (might be due to invalid IL or missing references)
		//IL_08c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0936: Unknown result type (might be due to invalid IL or missing references)
		//IL_09a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a16: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a86: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bb5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c49: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c93: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ce2: Unknown result type (might be due to invalid IL or missing references)
		//IL_0d31: Unknown result type (might be due to invalid IL or missing references)
		//IL_0d80: Unknown result type (might be due to invalid IL or missing references)
		//IL_0dcf: Unknown result type (might be due to invalid IL or missing references)
		//IL_0e19: Unknown result type (might be due to invalid IL or missing references)
		//IL_0e63: Unknown result type (might be due to invalid IL or missing references)
		//IL_0eb2: Unknown result type (might be due to invalid IL or missing references)
		//IL_0eb7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0f0b: Unknown result type (might be due to invalid IL or missing references)
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		UseStoppingTime = GUILayout.Toggle(UseStoppingTime, "Maximum Stopping Time", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		MaxStoppingTime.Text = GUILayout.TextField(MaxStoppingTime.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(60f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		UseFlipTime = GUILayout.Toggle(UseFlipTime, "Minimum Flip Time", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		MinFlipTime.Text = GUILayout.TextField(MinFlipTime.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(60f)
		});
		GUILayout.EndHorizontal();
		if (!UseStoppingTime)
		{
			UseFlipTime = false;
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		UseControlRange = GUILayout.Toggle(UseControlRange, Localizer.Format("#MechJeb_HybridController_checkbox2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		RollControlRange.Text = GUILayout.TextField(RollControlRange.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(60f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		GUILayout.Label("Velocity", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.Label("Position", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Kp", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		VelKp.Text = GUILayout.TextField(VelKp.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		PosKp.Text = GUILayout.TextField(PosKp.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Ti", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		VelTi.Text = GUILayout.TextField(VelTi.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		PosTi.Text = GUILayout.TextField(PosTi.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Td", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		VelTd.Text = GUILayout.TextField(VelTd.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		PosTd.Text = GUILayout.TextField(PosTd.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("N", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		VelN.Text = GUILayout.TextField(VelN.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		PosN.Text = GUILayout.TextField(PosN.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("B", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		VelB.Text = GUILayout.TextField(VelB.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		PosB.Text = GUILayout.TextField(PosB.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("C", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		VelC.Text = GUILayout.TextField(VelC.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		PosC.Text = GUILayout.TextField(PosC.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Deadband", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		VelDeadband.Text = GUILayout.TextField(VelDeadband.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		PosDeadband.Text = GUILayout.TextField(PosDeadband.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("SmoothIn", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		VelSmoothIn.Text = GUILayout.TextField(VelSmoothIn.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		PosSmoothIn.Text = GUILayout.TextField(PosSmoothIn.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("SmoothOut", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		VelSmoothOut.Text = GUILayout.TextField(VelSmoothOut.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		PosSmoothOut.Text = GUILayout.TextField(PosSmoothOut.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("", (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(100f)
		});
		VelClegg = GUILayout.Toggle(VelClegg, Localizer.Format("Clegg"), (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		PosClegg = GUILayout.Toggle(PosClegg, Localizer.Format("Clegg"), (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutNoExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Pos PTerm", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrintSci(new Vector3d(_posPID[0].PTerm, _posPID[1].PTerm, _posPID[2].PTerm)), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Pos ITerm", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrintSci(new Vector3d(_posPID[0].ITerm, _posPID[1].ITerm, _posPID[2].ITerm)), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Pos DTerm", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrintSci(new Vector3d(_posPID[0].DTerm, _posPID[1].DTerm, _posPID[2].DTerm)), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Vel PTerm", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrintSci(new Vector3d(_velPID[0].PTerm, _velPID[1].PTerm, _velPID[2].PTerm)), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Vel ITerm", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrintSci(new Vector3d(_velPID[0].ITerm, _velPID[1].ITerm, _velPID[2].ITerm)), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Vel DTerm", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrintSci(new Vector3d(_velPID[0].DTerm, _velPID[1].DTerm, _velPID[2].DTerm)), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Smooth Torque", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		SmoothTorque.Text = GUILayout.TextField(SmoothTorque.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Soften", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		Soften.Text = GUILayout.TextField(Soften.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GuiUtils.LayoutWidth(50f)
		});
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(Localizer.Format("Reset Tuning Values"), Array.Empty<GUILayoutOption>()))
		{
			Defaults();
		}
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Current", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
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
		GUILayout.Label("TargetOmega", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_targetOmega), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Omega", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_vessel.angularVelocityD), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_HybridController_label2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_actuation), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
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
		GUILayout.Label("TargetAlpha", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_targetAlpha), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("MaxAlpha", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_maxAlpha), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("MOI", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(Vector3d.op_Implicit(_vessel.MOI)), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("Response Speed", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(Ac.VesselState.TorqueResponseSpeed), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
	}
}
