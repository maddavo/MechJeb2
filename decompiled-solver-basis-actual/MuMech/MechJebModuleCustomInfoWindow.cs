using System;
using System.Collections.Generic;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleCustomInfoWindow : DisplayModule
{
	[Persistent(pass = 4)]
	public string title = Localizer.Format("#MechJeb_WindowEd_CustomInfoWindow_title");

	[Persistent(collectionIndex = "InfoItem", pass = 4)]
	public List<InfoItem> items = new List<InfoItem>();

	[Persistent(pass = 4)]
	public bool isCompact;

	[Persistent(pass = 4)]
	public Color backgroundColor = new Color(0f, 0f, 0f, 1f);

	[Persistent(pass = 4)]
	public Color text = new Color(1f, 1f, 1f, 1f);

	public Texture2D background;

	private GUISkin localSkin;

	private TimeSpan refreshInterval = TimeSpan.FromSeconds(0.1);

	private DateTime lastRefresh = DateTime.MinValue;

	[Persistent(pass = 4)]
	public EditableInt refreshRate = 10;

	public bool IsCompact
	{
		get
		{
			return isCompact;
		}
		set
		{
			Dirty = isCompact != value;
			isCompact = value;
		}
	}

	public void UpdateRefreshRate()
	{
		refreshInterval = TimeSpan.FromSeconds(1.0 / (double)(int)refreshRate);
	}

	public override void OnDestroy()
	{
		if (Object.op_Implicit((Object)(object)background))
		{
			Object.Destroy((Object)(object)background);
		}
		base.OnDestroy();
	}

	public override void OnSave(ConfigNode local, ConfigNode type, ConfigNode global)
	{
	}

	public virtual void UpdateWindowItems()
	{
		DateTime now = DateTime.Now;
		if (now - lastRefresh < refreshInterval)
		{
			return;
		}
		foreach (InfoItem item in items)
		{
			if (HighLogic.LoadedSceneIsEditor ? item.showInEditor : item.showInFlight)
			{
				item.UpdateItem();
			}
		}
		lastRefresh = now;
	}

	private void RefreshRateGUI()
	{
		int num = refreshRate;
		if (GuiUtils.ShowAdvancedWindowSettings)
		{
			GuiUtils.SimpleTextBox("Update Interval", refreshRate, "Hz");
		}
		if (num != (int)refreshRate)
		{
			refreshRate = Math.Max(refreshRate, 1);
			UpdateRefreshRate();
		}
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Invalid comparison between Unknown and I4
		//IL_00e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0136: Unknown result type (might be due to invalid IL or missing references)
		GUI.skin = (isCompact ? GuiUtils.CompactSkin : GuiUtils.Skin);
		GUI.contentColor = text;
		if ((int)Event.current.type == 8)
		{
			UpdateWindowItems();
		}
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		foreach (InfoItem item in items)
		{
			if (HighLogic.LoadedSceneIsEditor ? item.showInEditor : item.showInFlight)
			{
				item.DrawItem();
			}
			else
			{
				GUILayout.Label(item.localizedName, Array.Empty<GUILayoutOption>());
			}
		}
		if (items.Count == 0)
		{
			GUILayout.Label(CachedLocalizer.Instance.MechJebWindowEdCustomInfoWindowLabel1, Array.Empty<GUILayoutOption>());
		}
		RefreshRateGUI();
		GUILayout.EndVertical();
		if (!base.IsOverlay && GUI.Button(new Rect(10f, 0f, 13f, 20f), "E", GuiUtils.YellowOnHover))
		{
			MechJebModuleCustomWindowEditor computerModule = Core.GetComputerModule<MechJebModuleCustomWindowEditor>();
			if (computerModule != null)
			{
				computerModule.Enabled = true;
				computerModule.editedWindow = this;
			}
		}
		if (!base.IsOverlay && GUI.Button(new Rect(25f, 0f, 13f, 20f), "C", GuiUtils.YellowOnHover))
		{
			MuUtils.SystemClipboard = ToSharingString();
			ScreenMessages.PostScreenMessage(Localizer.Format("#MechJeb_WindowEd_CustomInfoWindow_Scrmsg1", new string[1] { GetName() }), 3f, (ScreenMessageStyle)2);
		}
		base.WindowGUI(windowID);
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(250f),
			GUILayout.Height(30f)
		};
	}

	public override void DrawGUI(bool inEditor)
	{
		Init();
		if (base.IsOverlay)
		{
			GUI.skin = localSkin;
		}
		base.DrawGUI(inEditor);
		if (base.IsOverlay)
		{
			GUI.skin = GuiUtils.Skin;
		}
	}

	public void Init()
	{
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Expected O, but got Unknown
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		if (!Object.op_Implicit((Object)(object)background))
		{
			background = new Texture2D(1, 1, (TextureFormat)5, false);
			background.SetPixel(0, 0, backgroundColor);
			background.Apply();
		}
		if (base.IsOverlay && !Object.op_Implicit((Object)(object)localSkin))
		{
			localSkin = Object.Instantiate<GUISkin>(GuiUtils.TransparentSkin);
			localSkin.window.normal.background = background;
			localSkin.window.onNormal.background = background;
		}
	}

	public string ToSharingString()
	{
		string text = "--- MechJeb Custom Window ---\n";
		text = text + "Name: " + GetName() + "\n";
		text = text + "Show in:" + (base.ShowInEditor ? " editor" : "") + (base.ShowInFlight ? " flight" : "") + "\n";
		for (int i = 0; i < items.Count; i++)
		{
			InfoItem infoItem = items[i];
			text = text + infoItem.id + "\n";
		}
		text += "-----------------------------\n";
		return text.Replace("\n", Environment.NewLine);
	}

	public void FromSharingString(string[] lines, List<InfoItem> registry)
	{
		if (lines.Length > 1 && lines[1].StartsWith("Name: "))
		{
			title = lines[1].Trim().Substring("Name: ".Length);
		}
		if (lines.Length > 2 && lines[2].StartsWith("Show in:"))
		{
			base.ShowInEditor = lines[2].Contains("editor");
			base.ShowInFlight = lines[2].Contains("flight");
		}
		for (int i = 3; i < lines.Length; i++)
		{
			string id = lines[i].Trim();
			InfoItem infoItem = registry.FirstOrDefault((InfoItem item) => item.id == id);
			if (infoItem != null)
			{
				items.Add(infoItem);
			}
		}
	}

	public override string GetName()
	{
		return title;
	}

	public override string IconName()
	{
		return title;
	}

	public MechJebModuleCustomInfoWindow(MechJebCore core)
		: base(core)
	{
	}//IL_0030: Unknown result type (might be due to invalid IL or missing references)
	//IL_0035: Unknown result type (might be due to invalid IL or missing references)
	//IL_004f: Unknown result type (might be due to invalid IL or missing references)
	//IL_0054: Unknown result type (might be due to invalid IL or missing references)

}
