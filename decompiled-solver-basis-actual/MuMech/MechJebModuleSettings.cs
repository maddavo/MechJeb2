using System;
using KSP.IO;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleSettings : DisplayModule
{
	[Persistent(pass = 4)]
	public int SkinId = 2;

	[Persistent(pass = 4)]
	public readonly EditableDouble UIScale = 1.0;

	[Persistent(pass = 4)]
	public bool DontUseDropDownMenu;

	[ToggleInfoItem("#MechJeb_hideBrakeOnEject", InfoItem.Category.Misc)]
	[Persistent(pass = 4)]
	public bool HideBrakeOnEject;

	[ToggleInfoItem("#MechJeb_useTitlebarDragging", InfoItem.Category.Misc)]
	[Persistent(pass = 4)]
	public bool UseTitlebarDragging;

	[ToggleInfoItem("#MechJeb_rssMode", InfoItem.Category.Misc)]
	[Persistent(pass = 4)]
	public bool RssMode;

	[Persistent(pass = 4)]
	public bool ShowAdvancedWindowSettings;

	public MechJebModuleSettings(MechJebCore core)
		: base(core)
	{
		base.ShowInEditor = true;
		base.ShowInFlight = true;
	}

	public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnLoad(local, type, global);
		GuiUtils.SetGUIScale(UIScale.Val);
		GuiUtils.DontUseDropDownMenu = DontUseDropDownMenu;
	}

	protected override void WindowGUI(int windowID)
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(Localizer.Format("#MechJeb_Settings_button1"), Array.Empty<GUILayoutOption>()))
		{
			FileInfo.CreateForType<MechJebCore>("mechjeb_settings_global.cfg", (Vessel)null).Delete();
			if ((Object)(object)base.Vessel != (Object)null && base.Vessel.vesselName != null)
			{
				FileInfo.CreateForType<MechJebCore>("mechjeb_settings_type_" + base.Vessel.vesselName + ".cfg", (Vessel)null).Delete();
			}
			Core.ReloadAllComputerModules();
			GuiUtils.SetGUIScale(1.0);
		}
		GUILayout.Label(Localizer.Format("#MechJeb_Settings_label1", new object[1] { (GuiUtils.SkinType)SkinId }), Array.Empty<GUILayoutOption>());
		if (((Object)(object)GuiUtils.Skin == (Object)null || SkinId != 1) && GUILayout.Button(Localizer.Format("#MechJeb_Settings_button2"), Array.Empty<GUILayoutOption>()))
		{
			GuiUtils.LoadSkin(GuiUtils.SkinType.MECH_JEB1);
			SkinId = 1;
		}
		if (((Object)(object)GuiUtils.Skin == (Object)null || SkinId != 0) && GUILayout.Button(Localizer.Format("#MechJeb_Settings_button3"), Array.Empty<GUILayoutOption>()))
		{
			GuiUtils.LoadSkin(GuiUtils.SkinType.DEFAULT);
			SkinId = 0;
		}
		if (((Object)(object)GuiUtils.Skin == (Object)null || SkinId != 2) && GUILayout.Button(Localizer.Format("#MechJeb_Settings_button4"), Array.Empty<GUILayoutOption>()))
		{
			GuiUtils.LoadSkin(GuiUtils.SkinType.COMPACT);
			SkinId = 2;
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_Settings_label2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		UIScale.Text = GUILayout.TextField(UIScale.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(60f) });
		GUILayout.EndHorizontal();
		GuiUtils.SetGUIScale(UIScale.Val);
		DontUseDropDownMenu = GUILayout.Toggle(DontUseDropDownMenu, Localizer.Format("#MechJeb_Settings_checkbox1"), Array.Empty<GUILayoutOption>());
		GuiUtils.DontUseDropDownMenu = DontUseDropDownMenu;
		ShowAdvancedWindowSettings = GUILayout.Toggle(ShowAdvancedWindowSettings, "Show Advanced Window Settings", Array.Empty<GUILayoutOption>());
		GuiUtils.ShowAdvancedWindowSettings = ShowAdvancedWindowSettings;
		MechJebModuleCustomWindowEditor computerModule = Core.GetComputerModule<MechJebModuleCustomWindowEditor>();
		computerModule.registry.Find((InfoItem i) => i.id == "Toggle:Settings.HideBrakeOnEject").DrawItem();
		computerModule.registry.Find((InfoItem i) => i.id == "Toggle:Settings.UseTitlebarDragging").DrawItem();
		computerModule.registry.Find((InfoItem i) => i.id == "Toggle:Menu.useAppLauncher").DrawItem();
		if (ToolbarManager.ToolbarAvailable || Core.GetComputerModule<MechJebModuleMenu>().useAppLauncher)
		{
			computerModule.registry.Find((InfoItem i) => i.id == "Toggle:Menu.hideButton").DrawItem();
		}
		computerModule.registry.Find((InfoItem i) => i.id == "General:Menu.MenuPosition").DrawItem();
		computerModule.registry.Find((InfoItem i) => i.id == "Toggle:Settings.RssMode").DrawItem();
		Core.Warp.activateSASOnWarp = GUILayout.Toggle(Core.Warp.activateSASOnWarp, Localizer.Format("#MechJeb_Settings_checkbox2"), Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_Settings_title");
	}

	public override string IconName()
	{
		return "Settings";
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(200f),
			GUILayout.Height(100f)
		};
	}
}
