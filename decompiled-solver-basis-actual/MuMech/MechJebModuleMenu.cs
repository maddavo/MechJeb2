using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using KSP.Localization;
using KSP.UI.Screens;
using UnityEngine;

namespace MuMech;

public class MechJebModuleMenu : DisplayModule
{
	public enum WindowStat
	{
		HIDDEN,
		MINIMIZED,
		NORMAL,
		OPENING,
		CLOSING
	}

	public enum WindowSide
	{
		LEFT,
		RIGHT,
		TOP,
		BOTTOM
	}

	private struct Button
	{
		public IButton button;

		public string texturePath;

		public string texturePathActive;
	}

	public class DisplayOrder : IComparer<DisplayModule>
	{
		public static readonly DisplayOrder instance = new DisplayOrder();

		private DisplayOrder()
		{
		}

		int IComparer<DisplayModule>.Compare(DisplayModule a, DisplayModule b)
		{
			bool flag = a is MechJebModuleCustomInfoWindow;
			bool flag2 = b is MechJebModuleCustomInfoWindow;
			if (!flag && flag2)
			{
				return -1;
			}
			if (flag && !flag2)
			{
				return 1;
			}
			return string.Compare(a.GetName(), b.GetName(), StringComparison.Ordinal);
		}
	}

	[Persistent(pass = 4)]
	public WindowStat windowStat;

	[Persistent(pass = 4)]
	public WindowSide windowSide = WindowSide.TOP;

	[Persistent(pass = 4)]
	public float windowProgr;

	[Persistent(pass = 4)]
	public float windowVPos = -185f;

	[Persistent(pass = 4)]
	public float windowHPos = 250f;

	[Persistent(pass = 4)]
	public int columns = 2;

	private const int colWidth = 200;

	public bool firstDraw = true;

	private bool movingButton;

	[ToggleInfoItem("#MechJeb_HideMenuButton", InfoItem.Category.Misc)]
	[Persistent(pass = 4)]
	public readonly bool hideButton;

	[ToggleInfoItem("#MechJeb_UseAppLauncher", InfoItem.Category.Misc)]
	[Persistent(pass = 4)]
	public readonly bool useAppLauncher = true;

	private static Dictionary<DisplayModule, Button> toolbarButtons;

	private static Dictionary<Action, Button> featureButtons;

	private const string Qmark = "MechJeb2/Icons/QMark";

	private IButton menuButton;

	private static ApplicationLauncherButton mjButton;

	private static GUIStyle toggleInactive;

	private static GUIStyle toggleActive;

	public Rect displayedPos
	{
		get
		{
			//IL_002f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0034: Unknown result type (might be due to invalid IL or missing references)
			//IL_0055: Unknown result type (might be due to invalid IL or missing references)
			//IL_005a: Unknown result type (might be due to invalid IL or missing references)
			//IL_0069: Unknown result type (might be due to invalid IL or missing references)
			//IL_006e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0077: Unknown result type (might be due to invalid IL or missing references)
			//IL_007c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0084: Unknown result type (might be due to invalid IL or missing references)
			//IL_0097: Unknown result type (might be due to invalid IL or missing references)
			//IL_009c: Unknown result type (might be due to invalid IL or missing references)
			//IL_00be: Unknown result type (might be due to invalid IL or missing references)
			//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d2: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
			//IL_00e0: Unknown result type (might be due to invalid IL or missing references)
			//IL_00e5: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
			//IL_00fd: Unknown result type (might be due to invalid IL or missing references)
			//IL_0102: Unknown result type (might be due to invalid IL or missing references)
			//IL_0124: Unknown result type (might be due to invalid IL or missing references)
			//IL_0129: Unknown result type (might be due to invalid IL or missing references)
			//IL_0144: Unknown result type (might be due to invalid IL or missing references)
			//IL_0149: Unknown result type (might be due to invalid IL or missing references)
			//IL_0153: Unknown result type (might be due to invalid IL or missing references)
			//IL_0158: Unknown result type (might be due to invalid IL or missing references)
			//IL_0161: Unknown result type (might be due to invalid IL or missing references)
			//IL_0166: Unknown result type (might be due to invalid IL or missing references)
			//IL_016e: Unknown result type (might be due to invalid IL or missing references)
			//IL_017e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0183: Unknown result type (might be due to invalid IL or missing references)
			//IL_01a5: Unknown result type (might be due to invalid IL or missing references)
			//IL_01aa: Unknown result type (might be due to invalid IL or missing references)
			//IL_01c5: Unknown result type (might be due to invalid IL or missing references)
			//IL_01ca: Unknown result type (might be due to invalid IL or missing references)
			//IL_01d5: Unknown result type (might be due to invalid IL or missing references)
			//IL_01da: Unknown result type (might be due to invalid IL or missing references)
			//IL_01e3: Unknown result type (might be due to invalid IL or missing references)
			//IL_01e8: Unknown result type (might be due to invalid IL or missing references)
			//IL_01f0: Unknown result type (might be due to invalid IL or missing references)
			Rect windowPos;
			switch (windowSide)
			{
			case WindowSide.LEFT:
			{
				float num20 = windowProgr - 1f;
				windowPos = base.WindowPos;
				float num21 = num20 * ((Rect)(ref windowPos)).width;
				float num22 = -100f - windowVPos;
				float num23 = GuiUtils.ScaledScreenHeight;
				windowPos = base.WindowPos;
				float num24 = Mathf.Clamp(num22, 0f, num23 - ((Rect)(ref windowPos)).height);
				windowPos = base.WindowPos;
				float width4 = ((Rect)(ref windowPos)).width;
				windowPos = base.WindowPos;
				return new Rect(num21, num24, width4, ((Rect)(ref windowPos)).height);
			}
			case WindowSide.RIGHT:
			{
				float num14 = GuiUtils.ScaledScreenWidth;
				float num15 = windowProgr;
				windowPos = base.WindowPos;
				float num16 = num14 - num15 * ((Rect)(ref windowPos)).width;
				float num17 = -100f - windowVPos;
				float num18 = GuiUtils.ScaledScreenHeight;
				windowPos = base.WindowPos;
				float num19 = Mathf.Clamp(num17, 0f, num18 - ((Rect)(ref windowPos)).height);
				windowPos = base.WindowPos;
				float width3 = ((Rect)(ref windowPos)).width;
				windowPos = base.WindowPos;
				return new Rect(num16, num19, width3, ((Rect)(ref windowPos)).height);
			}
			case WindowSide.TOP:
			{
				float num8 = GuiUtils.ScaledScreenWidth - 50;
				windowPos = base.WindowPos;
				float num9 = num8 - ((Rect)(ref windowPos)).width * 0.5f - windowHPos;
				float num10 = GuiUtils.ScaledScreenWidth;
				windowPos = base.WindowPos;
				float num11 = Mathf.Clamp(num9, 0f, num10 - ((Rect)(ref windowPos)).width);
				float num12 = windowProgr - 1f;
				windowPos = base.WindowPos;
				float num13 = num12 * ((Rect)(ref windowPos)).height;
				windowPos = base.WindowPos;
				float width2 = ((Rect)(ref windowPos)).width;
				windowPos = base.WindowPos;
				return new Rect(num11, num13, width2, ((Rect)(ref windowPos)).height);
			}
			default:
			{
				float num = GuiUtils.ScaledScreenWidth - 50;
				windowPos = base.WindowPos;
				float num2 = num - ((Rect)(ref windowPos)).width * 0.5f - windowHPos;
				float num3 = GuiUtils.ScaledScreenWidth;
				windowPos = base.WindowPos;
				float num4 = Mathf.Clamp(num2, 0f, num3 - ((Rect)(ref windowPos)).width);
				float num5 = GuiUtils.ScaledScreenHeight;
				float num6 = windowProgr;
				windowPos = base.WindowPos;
				float num7 = num5 - num6 * ((Rect)(ref windowPos)).height;
				windowPos = base.WindowPos;
				float width = ((Rect)(ref windowPos)).width;
				windowPos = base.WindowPos;
				return new Rect(num4, num7, width, ((Rect)(ref windowPos)).height);
			}
			}
		}
	}

	public Rect buttonPos
	{
		get
		{
			//IL_003a: Unknown result type (might be due to invalid IL or missing references)
			//IL_003f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0058: Unknown result type (might be due to invalid IL or missing references)
			//IL_007f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0084: Unknown result type (might be due to invalid IL or missing references)
			//IL_009e: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d1: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d6: Unknown result type (might be due to invalid IL or missing references)
			//IL_00e9: Unknown result type (might be due to invalid IL or missing references)
			//IL_0122: Unknown result type (might be due to invalid IL or missing references)
			//IL_0127: Unknown result type (might be due to invalid IL or missing references)
			//IL_0141: Unknown result type (might be due to invalid IL or missing references)
			Rect windowPos;
			switch (windowSide)
			{
			case WindowSide.LEFT:
			{
				float num8 = Mathf.Clamp(windowVPos, (float)(-GuiUtils.ScaledScreenHeight), -100f);
				windowPos = base.WindowPos;
				return new Rect(num8, ((Rect)(ref windowPos)).width * windowProgr, 100f, 25f);
			}
			case WindowSide.RIGHT:
			{
				float num6 = Mathf.Clamp(windowVPos, (float)(-GuiUtils.ScaledScreenHeight), -100f);
				float num7 = GuiUtils.ScaledScreenWidth - 25;
				windowPos = base.WindowPos;
				return new Rect(num6, num7 - ((Rect)(ref windowPos)).width * windowProgr, 100f, 25f);
			}
			case WindowSide.TOP:
			{
				float num4 = Mathf.Clamp((float)GuiUtils.ScaledScreenWidth - windowHPos - 100f, 0f, (float)(GuiUtils.ScaledScreenWidth - 50));
				float num5 = windowProgr;
				windowPos = base.WindowPos;
				return new Rect(num4, num5 * ((Rect)(ref windowPos)).height, 100f, 25f);
			}
			default:
			{
				float num = Mathf.Clamp((float)GuiUtils.ScaledScreenWidth - windowHPos - 100f, 0f, (float)(GuiUtils.ScaledScreenWidth - 50));
				float num2 = GuiUtils.ScaledScreenHeight;
				float num3 = windowProgr;
				windowPos = base.WindowPos;
				return new Rect(num, num2 - num3 * ((Rect)(ref windowPos)).height - 25f, 100f, 25f);
			}
			}
		}
	}

	public bool HideMenuButton
	{
		get
		{
			if (ToolbarManager.ToolbarAvailable || useAppLauncher)
			{
				return hideButton;
			}
			return false;
		}
	}

	public MechJebModuleMenu(MechJebCore core)
		: base(core)
	{
		Priority = -1000;
		base.Enabled = true;
		Hidden = true;
		base.ShowInFlight = true;
		base.ShowInEditor = true;
		if (toolbarButtons == null)
		{
			toolbarButtons = new Dictionary<DisplayModule, Button>();
		}
		if (featureButtons == null)
		{
			featureButtons = new Dictionary<Action, Button>();
		}
	}

	[GeneralInfoItem("#MechJeb_MenuPosition", InfoItem.Category.Misc)]
	private void MenuPosition()
	{
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(windowSide.ToString(), Array.Empty<GUILayoutOption>()))
		{
			switch (windowSide)
			{
			case WindowSide.LEFT:
				windowSide = WindowSide.RIGHT;
				break;
			case WindowSide.RIGHT:
				windowSide = WindowSide.TOP;
				break;
			case WindowSide.TOP:
				windowSide = WindowSide.BOTTOM;
				break;
			case WindowSide.BOTTOM:
				windowSide = WindowSide.LEFT;
				break;
			default:
				windowSide = WindowSide.RIGHT;
				break;
			}
		}
		if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			columns--;
		}
		GUILayout.Label(columns.ToString(), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			columns++;
		}
		GUILayout.EndHorizontal();
		if (columns < 1)
		{
			columns = 1;
		}
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Expected O, but got Unknown
		//IL_0062: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_0068: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Unknown result type (might be due to invalid IL or missing references)
		//IL_0083: Expected O, but got Unknown
		//IL_0097: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a3: Unknown result type (might be due to invalid IL or missing references)
		if (HideMenuButton && GUI.Button(new Rect(2f, 2f, 16f, 16f), ""))
		{
			ShowHideWindow();
		}
		if (toggleInactive == null)
		{
			toggleInactive = new GUIStyle(GUI.skin.toggle);
			GUIStyleState normal = toggleInactive.normal;
			Color textColor = (toggleInactive.onNormal.textColor = Color.white);
			normal.textColor = textColor;
			toggleActive = new GUIStyle(toggleInactive);
			GUIStyleState normal2 = toggleActive.normal;
			textColor = (toggleActive.onNormal.textColor = Color.green);
			normal2.textColor = textColor;
		}
		List<DisplayModule> displayModules = Core.GetDisplayModules(DisplayOrder.instance);
		int num = 0;
		int num2 = Mathf.CeilToInt((float)(displayModules.Count((DisplayModule d) => !d.Hidden && d.ShowInCurrentScene) + 1) / (float)columns);
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		foreach (DisplayModule item in displayModules)
		{
			if (!item.Hidden && item.ShowInCurrentScene)
			{
				if (num == num2)
				{
					GUILayout.EndVertical();
					GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
					num = 0;
				}
				item.Enabled = GUILayout.Toggle(item.Enabled, item.GetName(), item.IsActive() ? toggleActive : toggleInactive, Array.Empty<GUILayoutOption>());
				num++;
			}
		}
		if (GUILayout.Button(Localizer.Format("#MechJeb_OnlineManualbutton"), Array.Empty<GUILayoutOption>()))
		{
			Application.OpenURL("https://github.com/MuMech/MechJeb2/wiki");
		}
		GUILayout.EndVertical();
		GUILayout.EndHorizontal();
		if (Core.someModuleAreLocked)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_ModuleAreLocked"), Array.Empty<GUILayoutOption>());
		}
	}

	public void SetupAppLauncher()
	{
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0056: Expected O, but got Unknown
		//IL_0056: Expected O, but got Unknown
		if (ApplicationLauncher.Ready)
		{
			if (useAppLauncher && (Object)(object)mjButton == (Object)null)
			{
				Texture2D texture = GameDatabase.Instance.GetTexture("MechJeb2/Icons/MJ2", false);
				mjButton = ApplicationLauncher.Instance.AddModApplication(new Callback(ShowHideMasterWindow), new Callback(ShowHideMasterWindow), (Callback)null, (Callback)null, (Callback)null, (Callback)null, (AppScenes)(-1), (Texture)(object)texture);
			}
			if (!useAppLauncher && (Object)(object)mjButton != (Object)null)
			{
				ApplicationLauncher.Instance.RemoveModApplication(mjButton);
				mjButton = null;
			}
		}
	}

	public void ShowHideMasterWindow()
	{
		FlightGlobals.ActiveVessel.GetMasterMechJeb().GetComputerModule<MechJebModuleMenu>().ShowHideWindow();
	}

	public void SetupToolBarButtons()
	{
		if (!ToolbarManager.ToolbarAvailable)
		{
			return;
		}
		SetupMainToolbarButton();
		foreach (DisplayModule module in from m in Core.GetDisplayModules(DisplayOrder.instance)
			where !m.Hidden
			select m)
		{
			Button value;
			if (!toolbarButtons.ContainsKey(module))
			{
				Debug.Log((object)("Create button for module " + module.GetName()));
				string cleanName = GetCleanName(module.IconName());
				string text = "MechJeb2/Icons/" + cleanName;
				string text2 = text + "_active";
				value = default(Button);
				value.button = ToolbarManager.Instance.add("MechJeb2", cleanName);
				if ((Object)(object)GameDatabase.Instance.GetTexture(text, false) == (Object)null)
				{
					value.texturePath = "MechJeb2/Icons/QMark";
					ComputerModule.Print("No icon for " + cleanName);
				}
				else
				{
					value.texturePath = text;
				}
				if ((Object)(object)GameDatabase.Instance.GetTexture(text2, false) == (Object)null)
				{
					value.texturePathActive = text;
				}
				else
				{
					value.texturePathActive = text2;
				}
				toolbarButtons[module] = value;
				value.button.ToolTip = "MechJeb " + module.GetName();
				value.button.OnClick += delegate
				{
					DisplayModule displayModule = FlightGlobals.ActiveVessel.GetMasterMechJeb().GetDisplayModules(DisplayOrder.instance).FirstOrDefault((DisplayModule m) => m == module);
					if (displayModule != null)
					{
						displayModule.Enabled = !displayModule.Enabled;
					}
				};
			}
			else
			{
				value = toolbarButtons[module];
			}
			value.button.Visible = module.ShowInCurrentScene;
			value.button.TexturePath = (module.IsActive() ? value.texturePathActive : value.texturePath);
		}
		if (featureButtons.Count != 0)
		{
			return;
		}
		MechJebModuleManeuverPlanner maneuverPlannerModule = Core.GetComputerModule<MechJebModuleManeuverPlanner>();
		if (HighLogic.LoadedSceneIsEditor || maneuverPlannerModule == null || maneuverPlannerModule.Hidden)
		{
			return;
		}
		CreateFeatureButton(maneuverPlannerModule, "Exec_Node", "MechJeb Execute Next Node", delegate
		{
			if (base.Vessel.patchedConicSolver.maneuverNodes.Count > 0 && Core.Node != null)
			{
				if (Core.Node.Enabled)
				{
					Core.Node.Abort();
				}
				else if (((Vector3d)(ref base.Vessel.patchedConicSolver.maneuverNodes[0].DeltaV)).magnitude > 0.0001)
				{
					Core.Node.ExecuteOneNode(maneuverPlannerModule);
				}
				else
				{
					ScreenMessages.PostScreenMessage("Maneuver burn vector not set", 3f);
				}
			}
			else
			{
				ScreenMessages.PostScreenMessage("No maneuver nodes", 2f);
			}
		}, () => base.Vessel.patchedConicSolver.maneuverNodes.Count > 0 && Core.Node != null && Core.Node.Enabled);
		CreateFeatureButton(maneuverPlannerModule, "Autostage_Once", "MechJeb Autostage Once", delegate
		{
			MechJebModuleThrustWindow computerModule = Core.GetComputerModule<MechJebModuleThrustWindow>();
			if (Core.Staging.Enabled && Core.Staging.AutostagingOnce)
			{
				if (Core.Staging.Users.Contains(computerModule))
				{
					Core.Staging.Users.Remove(computerModule);
					computerModule.autostageSavedState = false;
				}
			}
			else
			{
				Core.Staging.AutostageOnce(computerModule);
			}
		}, () => Core.Staging.Enabled && Core.Staging.AutostagingOnce);
		CreateFeatureButton(maneuverPlannerModule, "Auto_Warp", "MechJeb Auto-warp", delegate
		{
			Core.Node.Autowarp = !Core.Node.Autowarp;
		}, () => Core.Node.Autowarp);
	}

	public void CreateFeatureButton(DisplayModule module, string nameId, string tooltip, ClickHandler onClick, Func<bool> isActive)
	{
		string text = "MechJeb2/Icons/" + nameId;
		string text2 = text + "_active";
		Button button = default(Button);
		button.button = ToolbarManager.Instance.add("MechJeb2", nameId);
		if ((Object)(object)GameDatabase.Instance.GetTexture(text, false) == (Object)null)
		{
			button.texturePath = "MechJeb2/Icons/QMark";
			ComputerModule.Print("No icon for " + nameId);
		}
		else
		{
			button.texturePath = text;
		}
		if ((Object)(object)GameDatabase.Instance.GetTexture(text2, false) == (Object)null)
		{
			button.texturePathActive = text;
		}
		else
		{
			button.texturePathActive = text2;
		}
		button.button.ToolTip = tooltip;
		button.button.OnClick += onClick;
		featureButtons.Add(delegate
		{
			button.button.TexturePath = (isActive() ? button.texturePathActive : button.texturePath);
		}, button);
		button.button.Visible = module.ShowInCurrentScene;
		button.button.TexturePath = button.texturePath;
	}

	public void SetupMainToolbarButton()
	{
		if (!ToolbarManager.ToolbarAvailable)
		{
			return;
		}
		if (menuButton == null)
		{
			menuButton = ToolbarManager.Instance.add("MechJeb2", "MechJeb2MenuButton");
			menuButton.ToolTip = "MechJeb2";
			menuButton.TexturePath = "MechJeb2/Icons/MJ2";
			menuButton.OnClick += delegate
			{
				ShowHideMasterWindow();
			};
		}
		menuButton.Visible = true;
	}

	private string GetCleanName(string name)
	{
		string str = " .:" + new string(Path.GetInvalidFileNameChars()) + new string(Path.GetInvalidPathChars());
		return new Regex("[" + Regex.Escape(str) + "]").Replace(name, "_");
	}

	public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		if (HighLogic.LoadedSceneIsGame)
		{
			ClearButtons();
			base.OnLoad(local, type, global);
		}
	}

	public override void OnDestroy()
	{
		ClearButtons();
		base.OnDestroy();
	}

	private void ClearButtons()
	{
		if ((Object)(object)mjButton != (Object)null && (Object)(object)((Component)mjButton).gameObject != (Object)null)
		{
			ApplicationLauncher.Instance.RemoveModApplication(mjButton);
			mjButton = null;
		}
		if (!ToolbarManager.ToolbarAvailable)
		{
			return;
		}
		foreach (Button value in toolbarButtons.Values)
		{
			if (value.button != null)
			{
				value.button.Destroy();
			}
		}
		toolbarButtons.Clear();
		foreach (Button value2 in featureButtons.Values)
		{
			if (value2.button != null)
			{
				value2.button.Destroy();
			}
		}
		featureButtons.Clear();
		if (menuButton != null)
		{
			menuButton.Destroy();
		}
	}

	public override void DrawGUI(bool inEditor)
	{
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_016b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0178: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b7: Expected O, but got Unknown
		//IL_01b2: Unknown result type (might be due to invalid IL or missing references)
		switch (windowStat)
		{
		case WindowStat.OPENING:
			windowProgr += Time.deltaTime;
			if (windowProgr >= 1f)
			{
				windowProgr = 1f;
				windowStat = WindowStat.NORMAL;
			}
			break;
		case WindowStat.CLOSING:
			windowProgr -= Time.deltaTime;
			if (windowProgr <= 0f)
			{
				windowProgr = 0f;
				windowStat = WindowStat.HIDDEN;
			}
			break;
		}
		GUI.depth = -100;
		GUI.SetNextControlName("MechJebOpen");
		Matrix4x4 matrix = GUI.matrix;
		if (windowSide == WindowSide.RIGHT || windowSide == WindowSide.LEFT)
		{
			GUI.matrix *= Matrix4x4.TRS(Vector3.zero, Quaternion.Euler(new Vector3(0f, 0f, -90f)), Vector3.one);
		}
		if (!HideMenuButton && GUI.RepeatButton(buttonPos, ((windowStat == WindowStat.HIDDEN) ^ (windowSide == WindowSide.TOP || windowSide == WindowSide.LEFT)) ? "▲ MechJeb ▲" : "▼ MechJeb ▼"))
		{
			if (Event.current.button == 0)
			{
				ShowHideWindow();
			}
			else if (!movingButton && Event.current.button == 1)
			{
				movingButton = true;
			}
		}
		GUI.matrix = matrix;
		GUI.depth = -99;
		if (windowStat != 0)
		{
			base.WindowPos = GUILayout.Window(GetType().FullName.GetHashCode(), displayedPos, new WindowFunction(WindowGUI), "MechJeb " + Core.version, (GUILayoutOption[])(object)new GUILayoutOption[2]
			{
				GuiUtils.LayoutWidth(200f),
				GUILayout.Height(20f)
			});
		}
		else
		{
			base.WindowPos = new Rect((float)GuiUtils.ScaledScreenWidth, (float)GuiUtils.ScaledScreenHeight, 0f, 0f);
		}
		GUI.depth = -98;
		if (firstDraw)
		{
			GUI.FocusControl("MechJebOpen");
			firstDraw = false;
		}
	}

	public void ShowHideWindow()
	{
		if (windowStat == WindowStat.HIDDEN)
		{
			windowStat = WindowStat.OPENING;
			windowProgr = 0f;
			firstDraw = true;
		}
		else if (windowStat == WindowStat.NORMAL)
		{
			windowStat = WindowStat.CLOSING;
			windowProgr = 1f;
		}
	}

	public void OnMenuUpdate()
	{
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		if (movingButton)
		{
			if (Input.GetMouseButton(1))
			{
				if (windowSide == WindowSide.RIGHT || windowSide == WindowSide.LEFT)
				{
					windowVPos = Mathf.Clamp(Input.mousePosition.y - (float)Screen.height - 50f, (float)(-Screen.width), 0f) / GuiUtils.Scale;
				}
				else
				{
					windowHPos = Mathf.Clamp((float)Screen.width - Input.mousePosition.x - 50f, 0f, (float)Screen.width) / GuiUtils.Scale;
				}
			}
			else if (Input.GetMouseButtonUp(1))
			{
				movingButton = false;
			}
		}
		if (HighLogic.LoadedSceneIsEditor || ((Object)(object)base.Vessel != (Object)null && base.Vessel.isActiveVessel))
		{
			SetupAppLauncher();
			SetupToolBarButtons();
		}
		foreach (KeyValuePair<Action, Button> featureButton in featureButtons)
		{
			featureButton.Key();
		}
	}
}
