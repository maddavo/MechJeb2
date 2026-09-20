using System;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

internal class MechJebModuleSmartRcs : DisplayModule
{
	public enum Target
	{
		OFF,
		ZERO_RVEL
	}

	public static readonly string[] TargetTexts = new string[2]
	{
		Localizer.Format("#MechJeb_SmartRcs_button1"),
		Localizer.Format("#MechJeb_SmartRcs_button2")
	};

	public Target target;

	private static GUIStyle btNormal;

	private static GUIStyle btActive;

	private static GUIStyle btAuto;

	[Persistent(pass = 4)]
	public bool autoDisableSmartRCS = true;

	[GeneralInfoItem("#MechJeb_DisableSmartRcsAutomatically", InfoItem.Category.Misc)]
	public void AutoDisableSmartRCS()
	{
		autoDisableSmartRCS = GUILayout.Toggle(autoDisableSmartRCS, Localizer.Format("#MechJeb_SmartRcs_checkbox1 "), Array.Empty<GUILayoutOption>());
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

	public MechJebModuleSmartRcs(MechJebCore core)
		: base(core)
	{
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
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d6: Expected O, but got Unknown
		//IL_012b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0135: Expected O, but got Unknown
		//IL_013f: Unknown result type (might be due to invalid IL or missing references)
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
		if (Core.RCS.Enabled && Core.RCS.Users.Count((object u) => !Equals(u)) > 0)
		{
			if (autoDisableSmartRCS)
			{
				target = Target.OFF;
				if (Core.RCS.Users.Contains(this))
				{
					Core.RCS.Users.Remove(this);
				}
			}
			GUILayout.Button(Localizer.Format("#MechJeb_SmartRcs_button3"), btAuto, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		}
		else if (Core.Target.Target == null)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_SmartRcs_label1"), Array.Empty<GUILayoutOption>());
		}
		else
		{
			GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
			TargetButton(Target.OFF);
			TargetButton(Target.ZERO_RVEL);
			GUILayout.EndVertical();
		}
		Core.RCS.rcsThrottle = GUILayout.Toggle(Core.RCS.rcsThrottle, Localizer.Format("#MechJeb_SmartRcs_checkbox2"), Array.Empty<GUILayoutOption>());
		Core.RCS.rcsForRotation = GUILayout.Toggle(Core.RCS.rcsForRotation, Localizer.Format("#MechJeb_SmartRcs_checkbox3"), Array.Empty<GUILayoutOption>());
		base.WindowGUI(windowID);
	}

	public void Engage()
	{
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		switch (target)
		{
		case Target.OFF:
			Core.RCS.Users.Remove(this);
			break;
		case Target.ZERO_RVEL:
			Core.RCS.Users.Add(this);
			Core.RCS.SetTargetRelative(Vector3d.zero);
			break;
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
		return Localizer.Format("#MechJeb_SmartRcs_title");
	}

	public override string IconName()
	{
		return "SmartRcs";
	}
}
