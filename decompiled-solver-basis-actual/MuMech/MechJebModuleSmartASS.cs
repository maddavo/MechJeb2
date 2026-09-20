using System;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleSmartASS : DisplayModule
{
	public enum Mode
	{
		ORBITAL,
		SURFACE,
		TARGET,
		ADVANCED,
		AUTO
	}

	public enum Target
	{
		OFF,
		KILLROT,
		NODE,
		SURFACE,
		PROGRADE,
		RETROGRADE,
		NORMAL_PLUS,
		NORMAL_MINUS,
		RADIAL_PLUS,
		RADIAL_MINUS,
		RELATIVE_PLUS,
		RELATIVE_MINUS,
		TARGET_PLUS,
		TARGET_MINUS,
		PARALLEL_PLUS,
		PARALLEL_MINUS,
		ADVANCED,
		AUTO,
		SURFACE_PROGRADE,
		SURFACE_RETROGRADE,
		HORIZONTAL_PLUS,
		HORIZONTAL_MINUS,
		VERTICAL_PLUS
	}

	public static Mode[] Target2Mode = new Mode[23]
	{
		Mode.ORBITAL,
		Mode.ORBITAL,
		Mode.ORBITAL,
		Mode.SURFACE,
		Mode.ORBITAL,
		Mode.ORBITAL,
		Mode.ORBITAL,
		Mode.ORBITAL,
		Mode.ORBITAL,
		Mode.ORBITAL,
		Mode.TARGET,
		Mode.TARGET,
		Mode.TARGET,
		Mode.TARGET,
		Mode.TARGET,
		Mode.TARGET,
		Mode.ADVANCED,
		Mode.AUTO,
		Mode.SURFACE,
		Mode.SURFACE,
		Mode.SURFACE,
		Mode.SURFACE,
		Mode.SURFACE
	};

	public static bool[] TargetIsMode = new bool[23]
	{
		true, true, true, false, false, false, false, false, false, false,
		false, false, false, false, false, false, false, true, false, false,
		false, false, false
	};

	public static readonly string[] ModeTexts = new string[5]
	{
		Localizer.Format("#MechJeb_SmartASS_button1"),
		Localizer.Format("#MechJeb_SmartASS_button2"),
		Localizer.Format("#MechJeb_SmartASS_button3"),
		Localizer.Format("#MechJeb_SmartASS_button4"),
		Localizer.Format("#MechJeb_SmartASS_button5")
	};

	public static string[] ScriptModeTexts = new string[5]
	{
		Localizer.Format("#MechJeb_SmartASS_button6"),
		Localizer.Format("#MechJeb_SmartASS_button7"),
		Localizer.Format("#MechJeb_SmartASS_button8"),
		Localizer.Format("#MechJeb_SmartASS_button9"),
		Localizer.Format("#MechJeb_SmartASS_button10")
	};

	public static readonly string[] TargetTexts = new string[23]
	{
		Localizer.Format("#MechJeb_SmartASS_button11"),
		Localizer.Format("#MechJeb_SmartASS_button12"),
		Localizer.Format("#MechJeb_SmartASS_button13"),
		Localizer.Format("#MechJeb_SmartASS_button14"),
		Localizer.Format("#MechJeb_SmartASS_button15"),
		Localizer.Format("#MechJeb_SmartASS_button16"),
		Localizer.Format("#MechJeb_SmartASS_button17"),
		Localizer.Format("#MechJeb_SmartASS_button18"),
		Localizer.Format("#MechJeb_SmartASS_button19"),
		Localizer.Format("#MechJeb_SmartASS_button20"),
		Localizer.Format("#MechJeb_SmartASS_button21"),
		Localizer.Format("#MechJeb_SmartASS_button22"),
		Localizer.Format("#MechJeb_SmartASS_button23"),
		Localizer.Format("#MechJeb_SmartASS_button24"),
		Localizer.Format("#MechJeb_SmartASS_button25"),
		Localizer.Format("#MechJeb_SmartASS_button26"),
		Localizer.Format("#MechJeb_SmartASS_button27"),
		Localizer.Format("#MechJeb_SmartASS_button28"),
		Localizer.Format("#MechJeb_SmartASS_button29"),
		Localizer.Format("#MechJeb_SmartASS_button30"),
		Localizer.Format("#MechJeb_SmartASS_button31"),
		Localizer.Format("#MechJeb_SmartASS_button32"),
		Localizer.Format("#MechJeb_SmartASS_button33")
	};

	public static string[] ScriptTargetTexts = new string[23]
	{
		Localizer.Format("#MechJeb_SmartASS_button34"),
		Localizer.Format("#MechJeb_SmartASS_button35"),
		Localizer.Format("#MechJeb_SmartASS_button36"),
		Localizer.Format("#MechJeb_SmartASS_button37"),
		Localizer.Format("#MechJeb_SmartASS_button38"),
		Localizer.Format("#MechJeb_SmartASS_button39"),
		Localizer.Format("#MechJeb_SmartASS_button40"),
		Localizer.Format("#MechJeb_SmartASS_button41"),
		Localizer.Format("#MechJeb_SmartASS_button42"),
		Localizer.Format("#MechJeb_SmartASS_button43"),
		Localizer.Format("#MechJeb_SmartASS_button44"),
		Localizer.Format("#MechJeb_SmartASS_button45"),
		Localizer.Format("#MechJeb_SmartASS_button46"),
		Localizer.Format("#MechJeb_SmartASS_button47"),
		Localizer.Format("#MechJeb_SmartASS_button48"),
		Localizer.Format("#MechJeb_SmartASS_button49"),
		Localizer.Format("#MechJeb_SmartASS_button50"),
		Localizer.Format("#MechJeb_SmartASS_button51"),
		Localizer.Format("#MechJeb_SmartASS_button52"),
		Localizer.Format("#MechJeb_SmartASS_button53"),
		Localizer.Format("#MechJeb_SmartASS_button54"),
		Localizer.Format("#MechJeb_SmartASS_button55"),
		Localizer.Format("#MechJeb_SmartASS_button56")
	};

	public static readonly string[] ReferenceTexts = Enum.GetNames(typeof(AttitudeReference));

	public static readonly string[] directionTexts = Enum.GetNames(typeof(Vector6.Direction));

	public static GUIStyle btNormal;

	public static GUIStyle btActive;

	public static GUIStyle btAuto;

	[Persistent(pass = 1)]
	public Mode mode;

	[Persistent(pass = 1)]
	public Target target;

	[Persistent(pass = 1)]
	public EditableDouble srfHdg = new EditableDouble(90.0);

	[Persistent(pass = 1)]
	public EditableDouble srfPit = new EditableDouble(90.0);

	[Persistent(pass = 1)]
	public EditableDouble srfRol = new EditableDouble(0.0);

	[Persistent(pass = 1)]
	public EditableDouble srfVelYaw = new EditableDouble(0.0);

	[Persistent(pass = 1)]
	public EditableDouble srfVelPit = new EditableDouble(0.0);

	[Persistent(pass = 1)]
	public EditableDouble srfVelRol = new EditableDouble(0.0);

	[Persistent(pass = 1)]
	public readonly EditableDouble rol = new EditableDouble(0.0);

	[Persistent(pass = 1)]
	public AttitudeReference advReference = AttitudeReference.SUN;

	[Persistent(pass = 1)]
	public Vector6.Direction advDirection = Vector6.Direction.DOWN;

	[Persistent(pass = 1)]
	public bool forceRol;

	[Persistent(pass = 1)]
	public bool forcePitch = true;

	[Persistent(pass = 1)]
	public bool forceYaw = true;

	[Persistent(pass = 4)]
	public bool autoDisableSmartASS = true;

	[Persistent(pass = 1)]
	public bool smoothControl;

	[Persistent(pass = 1)]
	public EditableDouble degreesPerSecond = new EditableDouble(10.0);

	private Vector3d targetDirection = Vector3d.zero;

	private Quaternion targetAttitude = Quaternion.identity;

	private AttitudeReference targetReference = AttitudeReference.ORBIT;

	private static readonly double LARGE_INCREMENT = 10.0;

	private Vector3d curDirection = Vector3d.zero;

	private Quaternion curAttitude = Quaternion.identity;

	[GeneralInfoItem("#MechJeb_DisableSmartACSAutomatically", InfoItem.Category.Misc)]
	public void AutoDisableSmartASS()
	{
		autoDisableSmartASS = GUILayout.Toggle(autoDisableSmartASS, Core.eduMode ? Localizer.Format("#MechJeb_SmartASS_checkbox1") : Localizer.Format("#MechJeb_SmartASS_checkbox2"), Array.Empty<GUILayoutOption>());
	}

	public MechJebModuleSmartASS(MechJebCore core)
		: base(core)
	{
	}//IL_00c5: Unknown result type (might be due to invalid IL or missing references)
	//IL_00ca: Unknown result type (might be due to invalid IL or missing references)
	//IL_00d0: Unknown result type (might be due to invalid IL or missing references)
	//IL_00d5: Unknown result type (might be due to invalid IL or missing references)
	//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
	//IL_00e7: Unknown result type (might be due to invalid IL or missing references)
	//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
	//IL_00f2: Unknown result type (might be due to invalid IL or missing references)


	protected void ModeButton(Mode bt)
	{
		if (GUILayout.Button(ModeTexts[(int)bt], (mode == bt) ? btActive : btNormal, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GUILayout.ExpandHeight(true)
		}))
		{
			mode = bt;
		}
	}

	protected void TargetButton(Target bt)
	{
		if (GUILayout.Button(TargetTexts[(int)bt], (target == bt) ? btActive : btNormal, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GUILayout.ExpandHeight(true)
		}))
		{
			target = bt;
			Engage();
		}
	}

	protected void TargetButtonNoEngage(Target bt)
	{
		if (GUILayout.Button(TargetTexts[(int)bt], (target == bt) ? btActive : btNormal, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutExpandWidth,
			GUILayout.ExpandHeight(true)
		}))
		{
			target = bt;
		}
	}

	protected void ForceRoll()
	{
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		bool flag = forceRol;
		forceRol = GUILayout.Toggle(forceRol, Localizer.Format("#MechJeb_SmartASS_checkbox3"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		if (flag != forceRol)
		{
			Engage();
		}
		rol.Text = GUILayout.TextField(rol.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(30f) });
		GUILayout.Label("°", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
	}

	public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnLoad(local, type, global);
		if (target != 0)
		{
			Engage(resetPID: false);
		}
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0020: Expected O, but got Unknown
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0060: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		//IL_0099: Unknown result type (might be due to invalid IL or missing references)
		//IL_009a: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c9: Expected O, but got Unknown
		//IL_00ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d8: Expected O, but got Unknown
		//IL_012d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0137: Expected O, but got Unknown
		//IL_0141: Unknown result type (might be due to invalid IL or missing references)
		bool hasSmoothControl = false;
		if (btNormal == null)
		{
			btNormal = new GUIStyle(GUI.skin.button);
			GUIStyleState normal = btNormal.normal;
			Color textColor = (btNormal.focused.textColor = Color.white);
			normal.textColor = textColor;
			GUIStyleState hover = btNormal.hover;
			textColor = (btNormal.active.textColor = Color.yellow);
			hover.textColor = textColor;
			GUIStyleState onNormal = btNormal.onNormal;
			GUIStyleState onFocused = btNormal.onFocused;
			GUIStyleState onHover = btNormal.onHover;
			Color val = (btNormal.onActive.textColor = Color.green);
			Color val3 = (onHover.textColor = val);
			textColor = (onFocused.textColor = val3);
			onNormal.textColor = textColor;
			btNormal.padding = new RectOffset(8, 8, 8, 8);
			btActive = new GUIStyle(btNormal);
			btActive.active = btActive.onActive;
			btActive.normal = btActive.onNormal;
			btActive.onFocused = btActive.focused;
			btActive.hover = btActive.onHover;
			btAuto = new GUIStyle(btNormal);
			btAuto.normal.textColor = Color.red;
			GUIStyle obj = btAuto;
			GUIStyle obj2 = btAuto;
			GUIStyle obj3 = btAuto;
			GUIStyle obj4 = btAuto;
			GUIStyle obj5 = btAuto;
			GUIStyle obj6 = btAuto;
			GUIStyleState val5 = (btAuto.hover = btAuto.normal);
			GUIStyleState val7 = (obj6.focused = val5);
			GUIStyleState val9 = (obj5.active = val7);
			GUIStyleState val11 = (obj4.onNormal = val9);
			GUIStyleState val13 = (obj3.onHover = val11);
			GUIStyleState onActive = (obj2.onFocused = val13);
			obj.onActive = onActive;
		}
		if (Core.Attitude.Enabled && Core.Attitude.Users.Count((object u) => !Equals(u)) > 0)
		{
			if (autoDisableSmartASS)
			{
				target = Target.OFF;
				if (Core.Attitude.Users.Contains(this))
				{
					Core.Attitude.Users.Remove(this);
				}
			}
			GUILayout.Button(Localizer.Format("#MechJeb_SmartASS_button57"), btAuto, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		}
		else
		{
			GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			TargetButton(Target.OFF);
			TargetButton(Target.KILLROT);
			if (base.Vessel.patchedConicsUnlocked())
			{
				TargetButton(Target.NODE);
			}
			else
			{
				GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[2]
				{
					GuiUtils.LayoutExpandWidth,
					GUILayout.ExpandHeight(true)
				});
			}
			GUILayout.EndHorizontal();
			GUILayout.Label(Localizer.Format("#MechJeb_SmartASS_label1"), Array.Empty<GUILayoutOption>());
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			ModeButton(Mode.ORBITAL);
			ModeButton(Mode.SURFACE);
			ModeButton(Mode.TARGET);
			ModeButton(Mode.ADVANCED);
			GUILayout.EndHorizontal();
			switch (mode)
			{
			case Mode.ORBITAL:
				GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
				TargetButton(Target.PROGRADE);
				TargetButton(Target.NORMAL_PLUS);
				TargetButton(Target.RADIAL_PLUS);
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
				TargetButton(Target.RETROGRADE);
				TargetButton(Target.NORMAL_MINUS);
				TargetButton(Target.RADIAL_MINUS);
				GUILayout.EndHorizontal();
				ForceRoll();
				break;
			case Mode.SURFACE:
			{
				double num = ((!GameSettings.MODIFIER_KEY.GetKey(false)) ? 1 : 5);
				GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
				TargetButton(Target.SURFACE_PROGRADE);
				TargetButton(Target.SURFACE_RETROGRADE);
				TargetButtonNoEngage(Target.SURFACE);
				GUILayout.EndHorizontal();
				GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
				TargetButton(Target.HORIZONTAL_PLUS);
				TargetButton(Target.HORIZONTAL_MINUS);
				TargetButton(Target.VERTICAL_PLUS);
				GUILayout.EndHorizontal();
				if (target == Target.SURFACE)
				{
					bool flag = false;
					GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
					forceYaw = GUILayout.Toggle(forceYaw, "", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
					GuiUtils.SimpleTextBox("HDG", srfHdg, "°", 37f);
					if (GUILayout.Button("--", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfHdg = (double)srfHdg - LARGE_INCREMENT;
						flag = true;
					}
					if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfHdg = (double)srfHdg - num;
						flag = true;
					}
					if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfHdg = (double)srfHdg + num;
						flag = true;
					}
					if (GUILayout.Button("++", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfHdg = (double)srfHdg + LARGE_INCREMENT;
						flag = true;
					}
					if (GUILayout.Button("0", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfHdg = 0.0;
						flag = true;
					}
					if (GUILayout.Button("90", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(35f) }))
					{
						srfHdg = 90.0;
						flag = true;
					}
					GUILayout.EndHorizontal();
					GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
					forcePitch = GUILayout.Toggle(forcePitch, "", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
					GuiUtils.SimpleTextBox("PIT", srfPit, "°", 37f);
					if (GUILayout.Button("--", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfPit = (double)srfPit - LARGE_INCREMENT;
						flag = true;
					}
					if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfPit = (double)srfPit - num;
						flag = true;
					}
					if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfPit = (double)srfPit + num;
						flag = true;
					}
					if (GUILayout.Button("++", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfPit = (double)srfPit + LARGE_INCREMENT;
						flag = true;
					}
					if (GUILayout.Button("0", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfPit = 0.0;
						flag = true;
					}
					if (GUILayout.Button("90", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(35f) }))
					{
						srfPit = 90.0;
						flag = true;
					}
					GUILayout.EndHorizontal();
					GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
					forceRol = GUILayout.Toggle(forceRol, "", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
					GuiUtils.SimpleTextBox("ROL", srfRol, "°", 37f);
					if (GUILayout.Button("--", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfRol = (double)srfRol - LARGE_INCREMENT;
						flag = true;
					}
					if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfRol = (double)srfRol - num;
						flag = true;
					}
					if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfRol = (double)srfRol + num;
						flag = true;
					}
					if (GUILayout.Button("++", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfRol = (double)srfRol + LARGE_INCREMENT;
						flag = true;
					}
					if (GUILayout.Button("0", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfRol = 0.0;
						flag = true;
					}
					if (GUILayout.Button("180", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(35f) }))
					{
						srfRol = 180.0;
						flag = true;
					}
					GUILayout.EndHorizontal();
					DisplaySmoothControlUI(ref hasSmoothControl);
					if (GUILayout.Button(Localizer.Format("#MechJeb_SmartASS_button58"), Array.Empty<GUILayoutOption>()))
					{
						Engage();
					}
					if (flag)
					{
						Engage(resetPID: false);
					}
					Core.Attitude.SetAxisControl(forcePitch, forceYaw, forceRol);
				}
				else if (target == Target.SURFACE_PROGRADE || target == Target.SURFACE_RETROGRADE)
				{
					bool flag2 = false;
					GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
					forceRol = GUILayout.Toggle(forceRol, "", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
					GuiUtils.SimpleTextBox("ROL", srfVelRol, "°", 37f);
					if (GUILayout.Button("--", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelRol = (double)srfVelRol - LARGE_INCREMENT;
						flag2 = true;
					}
					if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelRol = (double)srfVelRol - num;
						flag2 = true;
					}
					if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelRol = (double)srfVelRol + num;
						flag2 = true;
					}
					if (GUILayout.Button("++", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelRol = (double)srfVelRol + LARGE_INCREMENT;
						flag2 = true;
					}
					if (GUILayout.Button("CUR", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelRol = 0.0 - base.VesselState.Roll;
						flag2 = true;
					}
					if (GUILayout.Button("0", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelRol = 0.0;
						flag2 = true;
					}
					GUILayout.EndHorizontal();
					GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
					forcePitch = GUILayout.Toggle(forcePitch, "", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
					GuiUtils.SimpleTextBox("PIT", srfVelPit, "°", 37f);
					if (GUILayout.Button("--", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelPit = (double)srfVelPit - LARGE_INCREMENT;
						flag2 = true;
					}
					if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelPit = (double)srfVelPit - num;
						flag2 = true;
					}
					if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelPit = (double)srfVelPit + num;
						flag2 = true;
					}
					if (GUILayout.Button("++", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelPit = (double)srfVelPit + LARGE_INCREMENT;
						flag2 = true;
					}
					if (GUILayout.Button("CUR", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelPit = base.VesselState.AoA;
						flag2 = true;
					}
					if (GUILayout.Button("0", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelPit = 0.0;
						flag2 = true;
					}
					GUILayout.EndHorizontal();
					GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
					forceYaw = GUILayout.Toggle(forceYaw, "", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
					GuiUtils.SimpleTextBox("YAW", srfVelYaw, "°", 37f);
					if (GUILayout.Button("--", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelYaw = (double)srfVelYaw - LARGE_INCREMENT;
						flag2 = true;
					}
					if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelYaw = (double)srfVelYaw - num;
						flag2 = true;
					}
					if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelYaw = (double)srfVelYaw + num;
						flag2 = true;
					}
					if (GUILayout.Button("++", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelYaw = (double)srfVelYaw + LARGE_INCREMENT;
						flag2 = true;
					}
					if (GUILayout.Button("CUR", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelYaw = 0.0 - base.VesselState.AoS;
						flag2 = true;
					}
					if (GUILayout.Button("0", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
					{
						srfVelYaw = 0.0;
						flag2 = true;
					}
					GUILayout.EndHorizontal();
					DisplaySmoothControlUI(ref hasSmoothControl);
					if (GUILayout.Button(Localizer.Format("#MechJeb_SmartASS_button58"), Array.Empty<GUILayoutOption>()))
					{
						Engage();
					}
					if (flag2)
					{
						Engage(resetPID: false);
					}
					Core.Attitude.SetAxisControl(forcePitch, forceYaw, forceRol);
				}
				break;
			}
			case Mode.TARGET:
				if (Core.Target.NormalTargetExists)
				{
					GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
					TargetButton(Target.TARGET_PLUS);
					TargetButton(Target.RELATIVE_PLUS);
					TargetButton(Target.PARALLEL_PLUS);
					GUILayout.EndHorizontal();
					GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
					TargetButton(Target.TARGET_MINUS);
					TargetButton(Target.RELATIVE_MINUS);
					TargetButton(Target.PARALLEL_MINUS);
					GUILayout.EndHorizontal();
					ForceRoll();
				}
				else
				{
					GUILayout.Label(Localizer.Format("#MechJeb_SmartASS_label2"), Array.Empty<GUILayoutOption>());
				}
				break;
			case Mode.ADVANCED:
				GUILayout.Label(Localizer.Format("#MechJeb_SmartASS_label3"), Array.Empty<GUILayoutOption>());
				advReference = (AttitudeReference)GuiUtils.ComboBox.Box((int)advReference, ReferenceTexts, this);
				GUILayout.Label(Localizer.Format("#MechJeb_SmartASS_label4"), Array.Empty<GUILayoutOption>());
				advDirection = (Vector6.Direction)GuiUtils.ComboBox.Box((int)advDirection, directionTexts, directionTexts);
				ForceRoll();
				if (GUILayout.Button(Localizer.Format("#MechJeb_SmartASS_button58"), btNormal, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
				{
					target = Target.ADVANCED;
					Engage();
				}
				break;
			}
			if (!hasSmoothControl)
			{
				smoothControl = false;
			}
			GUILayout.EndVertical();
		}
		base.WindowGUI(windowID);
	}

	private void DisplaySmoothControlUI(ref bool hasSmoothControl)
	{
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		hasSmoothControl = true;
		smoothControl = GUILayout.Toggle(smoothControl, "Smooth Control:", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GuiUtils.SimpleTextBox("", degreesPerSecond, "°/s", 40f);
		GUILayout.EndHorizontal();
	}

	public void Engage(bool resetPID = true)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_010b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0110: Unknown result type (might be due to invalid IL or missing references)
		//IL_0115: Unknown result type (might be due to invalid IL or missing references)
		//IL_0128: Unknown result type (might be due to invalid IL or missing references)
		//IL_012d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0132: Unknown result type (might be due to invalid IL or missing references)
		//IL_0137: Unknown result type (might be due to invalid IL or missing references)
		//IL_013f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0144: Unknown result type (might be due to invalid IL or missing references)
		//IL_014c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0151: Unknown result type (might be due to invalid IL or missing references)
		//IL_0159: Unknown result type (might be due to invalid IL or missing references)
		//IL_015e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0166: Unknown result type (might be due to invalid IL or missing references)
		//IL_016b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0173: Unknown result type (might be due to invalid IL or missing references)
		//IL_0178: Unknown result type (might be due to invalid IL or missing references)
		//IL_0180: Unknown result type (might be due to invalid IL or missing references)
		//IL_0185: Unknown result type (might be due to invalid IL or missing references)
		//IL_018d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0192: Unknown result type (might be due to invalid IL or missing references)
		//IL_019a: Unknown result type (might be due to invalid IL or missing references)
		//IL_019f: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_0208: Unknown result type (might be due to invalid IL or missing references)
		//IL_020d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0220: Unknown result type (might be due to invalid IL or missing references)
		//IL_0225: Unknown result type (might be due to invalid IL or missing references)
		//IL_022a: Unknown result type (might be due to invalid IL or missing references)
		//IL_023c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0241: Unknown result type (might be due to invalid IL or missing references)
		//IL_0246: Unknown result type (might be due to invalid IL or missing references)
		//IL_024b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0266: Unknown result type (might be due to invalid IL or missing references)
		//IL_026b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0284: Unknown result type (might be due to invalid IL or missing references)
		//IL_0289: Unknown result type (might be due to invalid IL or missing references)
		//IL_028e: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_02aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_02af: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_02bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_02cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_0329: Unknown result type (might be due to invalid IL or missing references)
		//IL_032a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0330: Unknown result type (might be due to invalid IL or missing references)
		//IL_0331: Unknown result type (might be due to invalid IL or missing references)
		//IL_02dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_02de: Unknown result type (might be due to invalid IL or missing references)
		//IL_043d: Unknown result type (might be due to invalid IL or missing references)
		//IL_043e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0354: Unknown result type (might be due to invalid IL or missing references)
		//IL_0369: Unknown result type (might be due to invalid IL or missing references)
		//IL_036e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0383: Unknown result type (might be due to invalid IL or missing references)
		//IL_0388: Unknown result type (might be due to invalid IL or missing references)
		//IL_038d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0392: Unknown result type (might be due to invalid IL or missing references)
		//IL_0397: Unknown result type (might be due to invalid IL or missing references)
		//IL_0399: Unknown result type (might be due to invalid IL or missing references)
		//IL_039e: Unknown result type (might be due to invalid IL or missing references)
		//IL_03a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_03a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_03a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_03a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_02eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_02fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_030d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0312: Unknown result type (might be due to invalid IL or missing references)
		//IL_0317: Unknown result type (might be due to invalid IL or missing references)
		//IL_031c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0321: Unknown result type (might be due to invalid IL or missing references)
		//IL_0322: Unknown result type (might be due to invalid IL or missing references)
		//IL_0327: Unknown result type (might be due to invalid IL or missing references)
		//IL_046b: Unknown result type (might be due to invalid IL or missing references)
		//IL_046c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0455: Unknown result type (might be due to invalid IL or missing references)
		//IL_03cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_03d2: Unknown result type (might be due to invalid IL or missing references)
		//IL_03d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_03da: Unknown result type (might be due to invalid IL or missing references)
		//IL_03df: Unknown result type (might be due to invalid IL or missing references)
		//IL_03b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_03ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_03c0: Unknown result type (might be due to invalid IL or missing references)
		//IL_03c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_03e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_03ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_0422: Unknown result type (might be due to invalid IL or missing references)
		//IL_0427: Unknown result type (might be due to invalid IL or missing references)
		//IL_0402: Unknown result type (might be due to invalid IL or missing references)
		Quaternion val = default(Quaternion);
		Vector3d val2 = Vector3d.zero;
		AttitudeReference attitudeReference = AttitudeReference.ORBIT;
		switch (target)
		{
		default:
			return;
		case Target.OFF:
			Core.Attitude.attitudeDeactivate();
			return;
		case Target.KILLROT:
			Core.Attitude.attitudeKILLROT = true;
			val = Quaternion.LookRotation(base.Part.vessel.GetTransform().up, -base.Part.vessel.GetTransform().forward);
			attitudeReference = AttitudeReference.INERTIAL;
			break;
		case Target.NODE:
			val2 = Vector3d.forward;
			attitudeReference = AttitudeReference.MANEUVER_NODE;
			break;
		case Target.SURFACE:
			val = Quaternion.AngleAxis((float)(double)srfHdg, Vector3.up) * Quaternion.AngleAxis(0f - (float)(double)srfPit, Vector3.right) * Quaternion.AngleAxis(0f - (float)(double)srfRol, Vector3.forward);
			attitudeReference = AttitudeReference.SURFACE_NORTH;
			break;
		case Target.PROGRADE:
			val2 = Vector3d.forward;
			attitudeReference = AttitudeReference.ORBIT;
			break;
		case Target.RETROGRADE:
			val2 = Vector3d.back;
			attitudeReference = AttitudeReference.ORBIT;
			break;
		case Target.NORMAL_PLUS:
			val2 = Vector3d.left;
			attitudeReference = AttitudeReference.ORBIT;
			break;
		case Target.NORMAL_MINUS:
			val2 = Vector3d.right;
			attitudeReference = AttitudeReference.ORBIT;
			break;
		case Target.RADIAL_PLUS:
			val2 = Vector3d.up;
			attitudeReference = AttitudeReference.ORBIT;
			break;
		case Target.RADIAL_MINUS:
			val2 = Vector3d.down;
			attitudeReference = AttitudeReference.ORBIT;
			break;
		case Target.RELATIVE_PLUS:
			val2 = Vector3d.forward;
			attitudeReference = AttitudeReference.RELATIVE_VELOCITY;
			break;
		case Target.RELATIVE_MINUS:
			val2 = Vector3d.back;
			attitudeReference = AttitudeReference.RELATIVE_VELOCITY;
			break;
		case Target.TARGET_PLUS:
			val2 = Vector3d.forward;
			attitudeReference = AttitudeReference.TARGET;
			break;
		case Target.TARGET_MINUS:
			val2 = Vector3d.back;
			attitudeReference = AttitudeReference.TARGET;
			break;
		case Target.PARALLEL_PLUS:
			val2 = Vector3d.forward;
			attitudeReference = AttitudeReference.TARGET_ORIENTATION;
			break;
		case Target.PARALLEL_MINUS:
			val2 = Vector3d.back;
			attitudeReference = AttitudeReference.TARGET_ORIENTATION;
			break;
		case Target.ADVANCED:
			val2 = Vector6.Directions[(int)advDirection];
			attitudeReference = advReference;
			break;
		case Target.SURFACE_PROGRADE:
			val = Quaternion.AngleAxis(0f - (float)(double)srfVelRol, Vector3.forward) * Quaternion.AngleAxis(0f - (float)(double)srfVelPit, Vector3.right) * Quaternion.AngleAxis((float)(double)srfVelYaw, Vector3.up);
			attitudeReference = AttitudeReference.SURFACE_VELOCITY;
			break;
		case Target.SURFACE_RETROGRADE:
			val = Quaternion.AngleAxis((float)(double)srfVelRol + 180f, Vector3.forward) * Quaternion.AngleAxis(0f - (float)(double)srfVelPit + 180f, Vector3.right) * Quaternion.AngleAxis((float)(double)srfVelYaw, Vector3.up);
			attitudeReference = AttitudeReference.SURFACE_VELOCITY;
			break;
		case Target.HORIZONTAL_PLUS:
			val2 = Vector3d.forward;
			attitudeReference = AttitudeReference.SURFACE_HORIZONTAL;
			break;
		case Target.HORIZONTAL_MINUS:
			val2 = Vector3d.back;
			attitudeReference = AttitudeReference.SURFACE_HORIZONTAL;
			break;
		case Target.VERTICAL_PLUS:
			val2 = Vector3d.up;
			attitudeReference = AttitudeReference.SURFACE_NORTH;
			break;
		case Target.AUTO:
			return;
		}
		if (forceRol && val2 != Vector3d.zero)
		{
			val = Quaternion.LookRotation(Vector3d.op_Implicit(val2), Vector3d.op_Implicit(Vector3d.up)) * Quaternion.AngleAxis(0f - (float)(double)rol, Vector3d.op_Implicit(Vector3d.forward));
			val2 = Vector3d.zero;
		}
		targetDirection = val2;
		targetAttitude = val;
		targetReference = attitudeReference;
		if (smoothControl)
		{
			QuaternionD val3 = Core.Attitude.attitudeGetReferenceRotation(attitudeReference);
			QuaternionD val4 = QuaternionD.LookRotation(Vector3d.op_Implicit(base.Part.vessel.GetTransform().up), Vector3d.op_Implicit(-base.Part.vessel.GetTransform().forward));
			QuaternionD val5 = QuaternionD.Inverse(val3) * val4;
			if (val2 != Vector3d.zero)
			{
				curDirection = Vector3d.forward;
				curAttitude = Quaternion.identity;
			}
			else
			{
				curDirection = Vector3d.zero;
				curAttitude = QuaternionD.op_Implicit(val5);
			}
			if (curDirection != Vector3d.zero)
			{
				Core.Attitude.attitudeTo(curDirection, targetReference, this);
			}
			else
			{
				Core.Attitude.attitudeTo(QuaternionD.op_Implicit(curAttitude), targetReference, this);
			}
		}
		else if (val2 != Vector3d.zero)
		{
			Core.Attitude.attitudeTo(val2, attitudeReference, this);
		}
		else
		{
			Core.Attitude.attitudeTo(QuaternionD.op_Implicit(val), attitudeReference, this);
		}
		if (resetPID)
		{
			Core.Attitude.Controller.Reset();
		}
	}

	public override void OnFixedUpdate()
	{
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		//IL_0131: Unknown result type (might be due to invalid IL or missing references)
		//IL_0136: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_0147: Unknown result type (might be due to invalid IL or missing references)
		//IL_014c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0116: Unknown result type (might be due to invalid IL or missing references)
		//IL_011c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0123: Unknown result type (might be due to invalid IL or missing references)
		//IL_0128: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00af: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ba: Unknown result type (might be due to invalid IL or missing references)
		base.OnFixedUpdate();
		if (!smoothControl || target == Target.OFF || !Core.Attitude.Users.Contains(this))
		{
			curDirection = targetDirection;
			curAttitude = targetAttitude;
			return;
		}
		float num = (float)(double)degreesPerSecond * Time.fixedDeltaTime;
		if (targetDirection != Vector3d.zero && curDirection != Vector3d.zero)
		{
			float num2 = (float)Vector3d.Angle(curDirection, targetDirection);
			if (num < num2 && num2 > 0.01f)
			{
				float num3 = Mathf.Clamp01(num / num2);
				curDirection = Vector3d.Slerp(curDirection, targetDirection, num3);
			}
			else
			{
				curDirection = targetDirection;
			}
			Core.Attitude.attitudeTo(curDirection, targetReference, this);
		}
		else
		{
			float num4 = Quaternion.Angle(curAttitude, targetAttitude);
			if (num < num4 && num4 > 0.01f)
			{
				float num5 = Mathf.Clamp01(num / num4);
				curAttitude = Quaternion.Slerp(curAttitude, targetAttitude, num5);
			}
			else
			{
				curAttitude = targetAttitude;
			}
			Core.Attitude.attitudeTo(QuaternionD.op_Implicit(curAttitude), targetReference, this);
		}
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(180f),
			GUILayout.Height(100f)
		};
	}

	public override string GetName()
	{
		if (!Core.eduMode)
		{
			return Localizer.Format("#MechJeb_SmartASS_title");
		}
		return Localizer.Format("#MechJeb_SmartACS_title");
	}

	public override string IconName()
	{
		if (!Core.eduMode)
		{
			return "Smart A.S.S.";
		}
		return "Smart A.C.S.";
	}
}
