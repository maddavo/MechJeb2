using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech.AttitudeControllers;

internal class MJAttitudeController : BaseAttitudeController
{
	private PIDControllerV3 _pid;

	private Vector3d _lastAct = Vector3d.zero;

	private Vector3d _pidAction;

	private Vector3d _error;

	[Persistent(pass = 4)]
	public bool TfAutoTune = true;

	private Vector3d _tfV = new Vector3d(0.3, 0.3, 0.3);

	[Persistent(pass = 4)]
	public Vector3 TfVec = new Vector3(0.3f, 0.3f, 0.3f);

	[Persistent(pass = 4)]
	public double TfMin = 0.1;

	[Persistent(pass = 4)]
	public double TfMax = 0.5;

	[Persistent(pass = 4)]
	public bool LowPassFilter = true;

	[Persistent(pass = 4)]
	public double KpFactor = 3.0;

	[Persistent(pass = 4)]
	public double KiFactor = 6.0;

	[Persistent(pass = 4)]
	public double KdFactor = 0.5;

	[Persistent(pass = 4)]
	public double Deadband = 0.0001;

	[Persistent(pass = 4)]
	public EditableDouble KWlimit = 0.15;

	private readonly Vector3d _defaultTfV = new Vector3d(0.3, 0.3, 0.3);

	private EditableDouble _uiTfX;

	private EditableDouble _uiTfY;

	private EditableDouble _uiTfZ;

	private EditableDouble _uiTfMin;

	private EditableDouble _uiTfMax;

	private EditableDouble _uiKpFactor;

	private EditableDouble _uiKiFactor;

	private EditableDouble _uiKdFactor;

	private EditableDouble _uiDeadband;

	public MJAttitudeController(MechJebModuleAttitudeController controller)
		: base(controller)
	{
	}//IL_0001: Unknown result type (might be due to invalid IL or missing references)
	//IL_0006: Unknown result type (might be due to invalid IL or missing references)
	//IL_002e: Unknown result type (might be due to invalid IL or missing references)
	//IL_0033: Unknown result type (might be due to invalid IL or missing references)
	//IL_0048: Unknown result type (might be due to invalid IL or missing references)
	//IL_004d: Unknown result type (might be due to invalid IL or missing references)
	//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
	//IL_00e8: Unknown result type (might be due to invalid IL or missing references)


	public override void OnStart()
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		_pid = new PIDControllerV3(Vector3d.zero, Vector3d.zero, Vector3d.zero, 1.0, -1.0);
		SetPIDParameters();
		_lastAct = Vector3d.zero;
		_uiTfX = new EditableDouble(_tfV.x);
		_uiTfY = new EditableDouble(_tfV.y);
		_uiTfZ = new EditableDouble(_tfV.z);
		_uiTfMin = new EditableDouble(TfMin);
		_uiTfMax = new EditableDouble(TfMax);
		_uiKpFactor = new EditableDouble(KpFactor);
		_uiKiFactor = new EditableDouble(KiFactor);
		_uiKdFactor = new EditableDouble(KdFactor);
		_uiDeadband = new EditableDouble(Deadband);
	}

	public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		base.OnLoad(local, type, global);
		_tfV = Vector3d.op_Implicit(TfVec);
	}

	public override void OnSave(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		TfVec = Vector3d.op_Implicit(_tfV);
		base.OnSave(local, type, global);
	}

	private void SetPIDParameters()
	{
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_0078: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f1: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = (TfAutoTune ? _tfV : _defaultTfV).InvertNoNaN();
		_pid.Kd = KdFactor * val;
		_pid.Kp = 1.0 / (KpFactor * Math.Sqrt(2.0)) * _pid.Kd;
		((Vector3d)(ref _pid.Kp)).Scale(val);
		_pid.Ki = 1.0 / (KiFactor * Math.Sqrt(2.0)) * _pid.Kp;
		((Vector3d)(ref _pid.Ki)).Scale(val);
		_pid.INTAccum = _pid.INTAccum.Clamp(-5.0, 5.0);
	}

	private void TuneTf(Vector3d torque)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0116: Unknown result type (might be due to invalid IL or missing references)
		//IL_011b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0122: Unknown result type (might be due to invalid IL or missing references)
		//IL_0133: Unknown result type (might be due to invalid IL or missing references)
		//IL_0138: Unknown result type (might be due to invalid IL or missing references)
		Vector3d val = default(Vector3d);
		((Vector3d)(ref val))._002Ector((torque.x != 0.0) ? (Ac.VesselState.MoI.x / torque.x) : 0.0, (torque.y != 0.0) ? (Ac.VesselState.MoI.y / torque.y) : 0.0, (torque.z != 0.0) ? (Ac.VesselState.MoI.z / torque.z) : 0.0);
		_tfV = 0.05 * val;
		Vector3d val2 = Vector3d.one + 2.0 * Ac.VesselState.TorqueReactionSpeed;
		((Vector3d)(ref _tfV)).Scale(val2);
		_tfV = _tfV.Clamp(2.0 * (double)TimeWarp.fixedDeltaTime, TfMax);
		_tfV = _tfV.Clamp(TfMin, TfMax);
	}

	public override void ResetConfig()
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		_tfV = _defaultTfV;
		TfMin = 0.1;
		TfMax = 0.5;
		KpFactor = 3.0;
		KiFactor = 6.0;
		KdFactor = 0.5;
		Deadband = 0.0001;
		KWlimit = 0.15;
	}

	public override void Reset()
	{
		_pid.Reset();
	}

	public override void DrivePre(FlightCtrlState s, out Vector3d act, out Vector3d deltaEuler)
	{
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		//IL_0088: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0093: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00be: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_010c: Unknown result type (might be due to invalid IL or missing references)
		//IL_012a: Unknown result type (might be due to invalid IL or missing references)
		//IL_013a: Unknown result type (might be due to invalid IL or missing references)
		//IL_013f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0150: Unknown result type (might be due to invalid IL or missing references)
		//IL_015b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0166: Unknown result type (might be due to invalid IL or missing references)
		//IL_016b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0170: Unknown result type (might be due to invalid IL or missing references)
		//IL_0186: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0202: Unknown result type (might be due to invalid IL or missing references)
		//IL_0207: Unknown result type (might be due to invalid IL or missing references)
		//IL_020b: Unknown result type (might be due to invalid IL or missing references)
		//IL_021d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0222: Unknown result type (might be due to invalid IL or missing references)
		//IL_0227: Unknown result type (might be due to invalid IL or missing references)
		//IL_022b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0253: Unknown result type (might be due to invalid IL or missing references)
		//IL_0275: Unknown result type (might be due to invalid IL or missing references)
		//IL_0297: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_02cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_0241: Unknown result type (might be due to invalid IL or missing references)
		//IL_0382: Unknown result type (might be due to invalid IL or missing references)
		//IL_0387: Unknown result type (might be due to invalid IL or missing references)
		//IL_0473: Unknown result type (might be due to invalid IL or missing references)
		//IL_0478: Unknown result type (might be due to invalid IL or missing references)
		//IL_047f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0484: Unknown result type (might be due to invalid IL or missing references)
		//IL_048b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0499: Unknown result type (might be due to invalid IL or missing references)
		//IL_049e: Unknown result type (might be due to invalid IL or missing references)
		Transform referenceTransform = Ac.Vessel.ReferenceTransform;
		Vector3d val = QuaternionD.op_Implicit(QuaternionExtensions.Inverse(((Component)referenceTransform).transform.rotation)) * Ac.RequestedAttitude * Vector3d.forward;
		double num = Math.Abs(Vector3d.Angle(Vector3d.up, val));
		Vector2d val2 = default(Vector2d);
		((Vector2d)(ref val2))._002Ector(val.x, val.z);
		val2 = ((Vector2d)(ref val2)).normalized * num;
		Vector3 val3 = Vector3.Cross(Vector3d.op_Implicit(Ac.RequestedAttitude * Vector3d.op_Implicit(Vector3.forward)), referenceTransform.up);
		QuaternionD val4 = QuaternionD.AngleAxis((double)(float)num, Vector3d.op_Implicit(val3)) * Ac.RequestedAttitude;
		float num2 = Vector3.Angle(referenceTransform.right, Vector3d.op_Implicit(val4 * Vector3d.op_Implicit(Vector3.right))) * (float)Math.Sign(Vector3.Dot(Vector3d.op_Implicit(val4 * Vector3d.op_Implicit(Vector3.right)), referenceTransform.forward));
		_error = new Vector3d((0.0 - val2.y) * (Math.PI / 180.0), (double)num2 * (Math.PI / 180.0), val2.x * (Math.PI / 180.0));
		((Vector3d)(ref _error)).Scale(Ac.AxisControl);
		Vector3d val5 = _error + Ac.inertia;
		((Vector3d)(ref val5))._002Ector(Math.Max(-Math.PI, Math.Min(Math.PI, val5.x)), Math.Max(-Math.PI, Math.Min(Math.PI, val5.y)), Math.Max(-Math.PI, Math.Min(Math.PI, val5.z)));
		Vector3d val6 = Vector3d.Scale(Ac.VesselState.MoI, Ac.torque.InvertNoNaN());
		((Vector3d)(ref val5)).Scale(val6);
		Vector3d omega = Vector3d.op_Implicit(Ac.Vessel.angularVelocity);
		((Vector3d)(ref omega)).Scale(val6);
		if (TfAutoTune)
		{
			TuneTf(Ac.torque);
		}
		SetPIDParameters();
		Vector3d wlimit = default(Vector3d);
		((Vector3d)(ref wlimit))._002Ector(Math.Sqrt(val6.x * Math.PI * (double)KWlimit), Math.Sqrt(val6.y * Math.PI * (double)KWlimit), Math.Sqrt(val6.z * Math.PI * (double)KWlimit));
		_pidAction = _pid.Compute(val5, omega, wlimit);
		_pidAction.x = ((Math.Abs(_pidAction.x) >= Deadband) ? _pidAction.x : 0.0);
		_pidAction.y = ((Math.Abs(_pidAction.y) >= Deadband) ? _pidAction.y : 0.0);
		_pidAction.z = ((Math.Abs(_pidAction.z) >= Deadband) ? _pidAction.z : 0.0);
		act = _lastAct;
		if (LowPassFilter)
		{
			act.x += (_pidAction.x - _lastAct.x) * (1.0 / (_tfV.x / (double)TimeWarp.fixedDeltaTime + 1.0));
			act.y += (_pidAction.y - _lastAct.y) * (1.0 / (_tfV.y / (double)TimeWarp.fixedDeltaTime + 1.0));
			act.z += (_pidAction.z - _lastAct.z) * (1.0 / (_tfV.z / (double)TimeWarp.fixedDeltaTime + 1.0));
		}
		else
		{
			act = _pidAction;
		}
		_lastAct = act;
		deltaEuler = _error * (180.0 / Math.PI);
	}

	public override void GUI()
	{
		//IL_0237: Unknown result type (might be due to invalid IL or missing references)
		//IL_0522: Unknown result type (might be due to invalid IL or missing references)
		//IL_0576: Unknown result type (might be due to invalid IL or missing references)
		//IL_05ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_0619: Unknown result type (might be due to invalid IL or missing references)
		//IL_0627: Unknown result type (might be due to invalid IL or missing references)
		//IL_067b: Unknown result type (might be due to invalid IL or missing references)
		//IL_06cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_0723: Unknown result type (might be due to invalid IL or missing references)
		//IL_0772: Unknown result type (might be due to invalid IL or missing references)
		//IL_07ed: Unknown result type (might be due to invalid IL or missing references)
		if (GUILayout.Button(Localizer.Format("#MechJeb_adv_reset_button"), Array.Empty<GUILayoutOption>()))
		{
			ResetConfig();
		}
		TfAutoTune = GUILayout.Toggle(TfAutoTune, Localizer.Format("#MechJeb_AttitudeController_checkbox1"), Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Space(20f);
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		if (!TfAutoTune)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label1"), Array.Empty<GUILayoutOption>());
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label3"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			_uiTfX.Text = GUILayout.TextField(_uiTfX.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
			{
				GuiUtils.LayoutExpandWidth,
				GuiUtils.LayoutWidth(40f)
			});
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label4"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			_uiTfY.Text = GUILayout.TextField(_uiTfY.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
			{
				GuiUtils.LayoutExpandWidth,
				GuiUtils.LayoutWidth(40f)
			});
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label5"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			_uiTfZ.Text = GUILayout.TextField(_uiTfZ.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
			{
				GuiUtils.LayoutExpandWidth,
				GuiUtils.LayoutWidth(40f)
			});
			GUILayout.EndHorizontal();
			_uiTfX = Math.Max(0.01, _uiTfX);
			_uiTfY = Math.Max(0.01, _uiTfY);
			_uiTfZ = Math.Max(0.01, _uiTfZ);
		}
		else
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label6"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label(MuUtils.PrettyPrint(_tfV), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label7"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_AttitudeController_label8"), _uiTfMin, "", 50f);
			_uiTfMin = Math.Max(_uiTfMin, 0.01);
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_AttitudeController_label9"), _uiTfMax, "", 50f);
			_uiTfMax = Math.Max(_uiTfMax, 0.01);
			GUILayout.EndHorizontal();
		}
		GUILayout.EndVertical();
		GUILayout.EndHorizontal();
		bool flag = GUILayout.Toggle(LowPassFilter, Localizer.Format("#MechJeb_AttitudeController_checkbox2"), Array.Empty<GUILayoutOption>());
		if (LowPassFilter != flag)
		{
			SetPIDParameters();
			LowPassFilter = flag;
		}
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_PIDF"), Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox("Kd = ", _uiKdFactor, " / Tf", 50f);
		_uiKdFactor = Math.Max(_uiKdFactor, 0.01);
		GuiUtils.SimpleTextBox("Kp = pid.Kd / (", _uiKpFactor, " * Math.Sqrt(2) * Tf)", 50f);
		_uiKpFactor = Math.Max(_uiKpFactor, 0.01);
		GuiUtils.SimpleTextBox("Ki = pid.Kp / (", _uiKiFactor, " * Math.Sqrt(2) * Tf)", 50f);
		_uiKiFactor = Math.Max(_uiKiFactor, 0.01);
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_AttitudeController_PIDFactor1"), _uiDeadband, "", 50f);
		Deadband = Math.Max(_uiDeadband, 0.0);
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_AttitudeController_label11"), KWlimit, "%");
		double num = KWlimit;
		num = (EditableDouble)GUILayout.HorizontalSlider((float)num, 0f, 1f, Array.Empty<GUILayoutOption>());
		if (Math.Round(Math.Abs(num - (double)KWlimit), 3) > 0.0)
		{
			KWlimit = Math.Round(num, 3);
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label12"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_pid.Kp), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label13"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_pid.Ki), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label14"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_pid.Kd), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label15"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_error * 57.295780181884766), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label16"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_pid.PropAct), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label17"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_pid.DerivativeAct), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label18"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_pid.INTAccum), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label19"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(MuUtils.PrettyPrint(_pidAction), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeController_label20"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label("|" + ((Vector3d)(ref Ac.inertia)).magnitude.ToString("F3") + "| " + MuUtils.PrettyPrint(Ac.inertia), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		if (!TfAutoTune)
		{
			if (_tfV.x != (double)_uiTfX || _tfV.y != (double)_uiTfY || _tfV.z != (double)_uiTfZ)
			{
				_tfV.x = _uiTfX;
				_tfV.y = _uiTfY;
				_tfV.z = _uiTfZ;
				SetPIDParameters();
			}
		}
		else if (TfMin != (double)_uiTfMin || TfMax != (double)_uiTfMax)
		{
			TfMin = _uiTfMin;
			TfMax = _uiTfMax;
			SetPIDParameters();
		}
		if (KpFactor != (double)_uiKpFactor || KiFactor != (double)_uiKiFactor || KdFactor != (double)_uiKdFactor)
		{
			KpFactor = _uiKpFactor;
			KiFactor = _uiKiFactor;
			KdFactor = _uiKdFactor;
			SetPIDParameters();
		}
	}
}
