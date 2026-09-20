using System;
using System.Collections.Generic;
using KSP.Localization;
using KSP.UI.Screens;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleTranslatron : DisplayModule
{
	public enum AbortStage
	{
		OFF,
		THRUSTOFF,
		DECOUPLE,
		BURNUP,
		LAND,
		LANDING
	}

	protected static readonly string[] trans_texts = new string[4]
	{
		Localizer.Format("#MechJeb_Translatron_off"),
		Localizer.Format("#MechJeb_Translatron_KEEP_OBT"),
		Localizer.Format("#MechJeb_Translatron_KEEP_SURF"),
		Localizer.Format("#MechJeb_Translatron_KEEP_VERT")
	};

	protected AbortStage abort;

	protected double burnUpTime;

	protected bool autoMode;

	[Persistent(pass = 1)]
	public EditableDouble trans_spd = new EditableDouble(0.0);

	private static GUIStyle buttonStyle;

	public MechJebModuleTranslatron(MechJebCore core)
		: base(core)
	{
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_Translatron_title");
	}

	public override string IconName()
	{
		return "Translatron";
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(130f) };
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Expected O, but got Unknown
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_003e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_0097: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_009e: Unknown result type (might be due to invalid IL or missing references)
		//IL_009f: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c7: Expected O, but got Unknown
		//IL_01fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0200: Unknown result type (might be due to invalid IL or missing references)
		//IL_0209: Unknown result type (might be due to invalid IL or missing references)
		//IL_020e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0220: Unknown result type (might be due to invalid IL or missing references)
		//IL_0151: Unknown result type (might be due to invalid IL or missing references)
		//IL_0111: Unknown result type (might be due to invalid IL or missing references)
		//IL_0116: Unknown result type (might be due to invalid IL or missing references)
		//IL_011f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0124: Unknown result type (might be due to invalid IL or missing references)
		//IL_0136: Unknown result type (might be due to invalid IL or missing references)
		//IL_04b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_04af: Unknown result type (might be due to invalid IL or missing references)
		//IL_04bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_04bc: Unknown result type (might be due to invalid IL or missing references)
		//IL_04c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_04c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_04c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_04cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_04cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_04d0: Unknown result type (might be due to invalid IL or missing references)
		//IL_04d7: Unknown result type (might be due to invalid IL or missing references)
		//IL_04d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_04da: Unknown result type (might be due to invalid IL or missing references)
		//IL_04e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_04e3: Unknown result type (might be due to invalid IL or missing references)
		//IL_04e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_04ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_04eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_04ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_04f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_04f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_04f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_04fa: Unknown result type (might be due to invalid IL or missing references)
		Color val;
		Color val3;
		Color textColor;
		if (buttonStyle == null)
		{
			buttonStyle = new GUIStyle(GUI.skin.button);
			GUIStyleState normal = buttonStyle.normal;
			textColor = (buttonStyle.focused.textColor = Color.white);
			normal.textColor = textColor;
			GUIStyleState hover = buttonStyle.hover;
			textColor = (buttonStyle.active.textColor = Color.yellow);
			hover.textColor = textColor;
			GUIStyleState onNormal = buttonStyle.onNormal;
			GUIStyleState onFocused = buttonStyle.onFocused;
			GUIStyleState onHover = buttonStyle.onHover;
			val = (buttonStyle.onActive.textColor = Color.green);
			val3 = (onHover.textColor = val);
			textColor = (onFocused.textColor = val3);
			onNormal.textColor = textColor;
			buttonStyle.padding = new RectOffset(8, 8, 8, 8);
		}
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		Rect windowPos;
		if (Core.Thrust.Users.Count > 1 && !Core.Thrust.Users.Contains(this))
		{
			if (!autoMode)
			{
				windowPos = base.WindowPos;
				float x = ((Rect)(ref windowPos)).x;
				windowPos = base.WindowPos;
				base.WindowPos = new Rect(x, ((Rect)(ref windowPos)).y, 10f, 10f);
				autoMode = true;
			}
			buttonStyle.normal.textColor = Color.red;
			GUIStyle obj = buttonStyle;
			GUIStyle obj2 = buttonStyle;
			GUIStyle obj3 = buttonStyle;
			GUIStyle obj4 = buttonStyle;
			GUIStyle obj5 = buttonStyle;
			GUIStyle obj6 = buttonStyle;
			GUIStyleState val5 = (buttonStyle.hover = buttonStyle.normal);
			GUIStyleState val7 = (obj6.focused = val5);
			GUIStyleState val9 = (obj5.active = val7);
			GUIStyleState val11 = (obj4.onNormal = val9);
			GUIStyleState val13 = (obj3.onHover = val11);
			GUIStyleState onActive = (obj2.onFocused = val13);
			obj.onActive = onActive;
			GUILayout.Button(Localizer.Format("#MechJeb_Trans_auto"), buttonStyle, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		}
		else
		{
			if (autoMode)
			{
				windowPos = base.WindowPos;
				float x2 = ((Rect)(ref windowPos)).x;
				windowPos = base.WindowPos;
				base.WindowPos = new Rect(x2, ((Rect)(ref windowPos)).y, 10f, 10f);
				autoMode = false;
			}
			MechJebModuleThrustController.TMode mode = (MechJebModuleThrustController.TMode)GUILayout.SelectionGrid((int)Core.Thrust.Tmode, trans_texts, 2, buttonStyle, Array.Empty<GUILayoutOption>());
			SetMode(mode);
			float num = ((!GameSettings.MODIFIER_KEY.GetKey(false)) ? 1 : 5);
			Core.Thrust.TransKillH = GUILayout.Toggle(Core.Thrust.TransKillH, Localizer.Format("#MechJeb_Trans_kill_h"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.BeginHorizontal((GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_Trans_spd"), trans_spd, "", 37f);
			bool flag = false;
			if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
			{
				trans_spd = (double)trans_spd - (double)num;
				flag = true;
			}
			if (GUILayout.Button("0", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
			{
				trans_spd = 0.0;
				flag = true;
			}
			if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
			{
				trans_spd = (double)trans_spd + (double)num;
				flag = true;
			}
			GUILayout.EndHorizontal();
			if (GUILayout.Button(Localizer.Format("#MechJeb_Trans_spd_act") + ":", buttonStyle, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }) || flag)
			{
				Core.Thrust.TransSpdAct = (float)trans_spd.Val;
				GUIUtility.keyboardControl = 0;
			}
		}
		if (Core.Thrust.Tmode != 0)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Trans_current_spd") + Statics.ToSI(Core.Thrust.TransSpdAct, 4, int.MaxValue) + "m/s", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		}
		GUILayout.FlexibleSpace();
		GUILayout.Label("Automation", GuiUtils.UpperCenterLabel, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUIStyleState normal3 = buttonStyle.normal;
		GUIStyleState focused = buttonStyle.focused;
		GUIStyleState hover2 = buttonStyle.hover;
		GUIStyleState active = buttonStyle.active;
		GUIStyleState onNormal2 = buttonStyle.onNormal;
		GUIStyleState onFocused2 = buttonStyle.onFocused;
		GUIStyleState onHover2 = buttonStyle.onHover;
		Color val16 = (buttonStyle.onActive.textColor = ((abort != 0) ? Color.red : Color.green));
		Color val18 = (onHover2.textColor = val16);
		Color val20 = (onFocused2.textColor = val18);
		Color val22 = (onNormal2.textColor = val20);
		val = (active.textColor = val22);
		val3 = (hover2.textColor = val);
		textColor = (focused.textColor = val3);
		normal3.textColor = textColor;
		if (GUILayout.Button((abort != 0) ? Localizer.Format("#MechJeb_Trans_NOPANIC") : Localizer.Format("#MechJeb_Trans_PANIC"), buttonStyle, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
		{
			PanicSwitch();
		}
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	public void SetMode(MechJebModuleThrustController.TMode newMode)
	{
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0060: Unknown result type (might be due to invalid IL or missing references)
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		MechJebModuleThrustController.TMode tmode = Core.Thrust.Tmode;
		Core.Thrust.Tmode = newMode;
		if (Core.Thrust.Tmode != tmode)
		{
			Core.Thrust.TransSpdAct = Convert.ToInt16(trans_spd);
			Rect windowPos = base.WindowPos;
			float x = ((Rect)(ref windowPos)).x;
			windowPos = base.WindowPos;
			base.WindowPos = new Rect(x, ((Rect)(ref windowPos)).y, 10f, 10f);
			if (Core.Thrust.Tmode == MechJebModuleThrustController.TMode.OFF)
			{
				Core.Thrust.Users.Remove(this);
			}
			else
			{
				Core.Thrust.Users.Add(this);
			}
		}
	}

	public void PanicSwitch()
	{
		if (abort != 0)
		{
			if (abort == AbortStage.LAND || abort == AbortStage.LANDING)
			{
				Core.GetComputerModule<MechJebModuleLandingAutopilot>().StopLanding();
			}
			else
			{
				Core.Thrust.ThrustOff();
				Core.Thrust.Users.Remove(this);
				Core.Attitude.attitudeDeactivate();
			}
			abort = AbortStage.OFF;
		}
		else
		{
			abort = AbortStage.THRUSTOFF;
			Core.Thrust.Users.Add(this);
		}
	}

	public void recursiveDecouple()
	{
		int num = StageManager.LastStage;
		for (int i = 0; i < base.Part.vessel.parts.Count; i++)
		{
			Part val = base.Part.vessel.parts[i];
			if (val.HasModule<ModuleEngines>() && val.inverseStage < num)
			{
				num = val.inverseStage;
			}
		}
		List<Part> list = new List<Part>();
		for (int j = 0; j < base.Part.vessel.parts.Count; j++)
		{
			Part val2 = base.Part.vessel.parts[j];
			if (val2.inverseStage > num && (val2.HasModule<ModuleDecouple>() || val2.HasModule<ModuleAnchoredDecoupler>()))
			{
				list.Add(val2);
			}
		}
		for (int k = 0; k < list.Count; k++)
		{
			list[k].force_activate();
		}
		if ((Object)(object)base.Part.vessel == (Object)(object)FlightGlobals.ActiveVessel)
		{
			StageManager.ActivateStage(num);
		}
	}

	public override void Drive(FlightCtrlState s)
	{
		//IL_0159: Unknown result type (might be due to invalid IL or missing references)
		//IL_016c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0177: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Attitude.Users.Contains(this) && Core.Thrust.TransKillH && Core.Thrust.Tmode != 0)
		{
			Core.Attitude.Users.Add(this);
		}
		if (Core.Attitude.Users.Contains(this) && (!Core.Thrust.TransKillH || Core.Thrust.Tmode == MechJebModuleThrustController.TMode.OFF))
		{
			Core.Attitude.Users.Remove(this);
		}
		if (abort != 0)
		{
			switch (abort)
			{
			case AbortStage.THRUSTOFF:
				FlightInputHandler.SetNeutralControls();
				s.mainThrottle = 0f;
				abort = AbortStage.DECOUPLE;
				break;
			case AbortStage.DECOUPLE:
				recursiveDecouple();
				abort = AbortStage.BURNUP;
				burnUpTime = Planetarium.GetUniversalTime();
				break;
			case AbortStage.BURNUP:
				if (Planetarium.GetUniversalTime() - burnUpTime < 2.0 || base.VesselState.SpeedVertical < 10.0)
				{
					Core.Thrust.Tmode = MechJebModuleThrustController.TMode.DIRECT;
					Core.Attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, this);
					double num = Math.Abs(Vector3d.Angle(base.VesselState.Up, base.VesselState.Forward));
					Core.Thrust.TransSpdAct = ((num < 90.0) ? 100 : 0);
				}
				else
				{
					abort = AbortStage.LAND;
				}
				break;
			case AbortStage.LAND:
				Core.Thrust.Users.Remove(this);
				Core.GetComputerModule<MechJebModuleLandingAutopilot>().LandUntargeted(this);
				abort = AbortStage.LANDING;
				break;
			case AbortStage.LANDING:
				if (base.Vessel.LandedOrSplashed)
				{
					abort = AbortStage.OFF;
				}
				break;
			}
		}
		base.Drive(s);
	}
}
