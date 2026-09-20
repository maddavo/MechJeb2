using System;
using UnityEngine;

namespace MuMech;

public class DisplayModule : ComputerModule
{
	public bool Hidden;

	[Persistent(pass = 4)]
	public Vector4 WindowVector = new Vector4(10f, 40f, 0f, 0f);

	[Persistent(pass = 4)]
	public Vector4 WindowVectorEditor = new Vector4(10f, 40f, 0f, 0f);

	[Persistent(pass = 4)]
	public bool showInFlight = true;

	[Persistent(pass = 4)]
	public bool showInEditor;

	[Persistent(pass = 4)]
	public bool IsOverlayConfig;

	[Persistent(pass = 4)]
	public bool LockedConfig;

	internal bool EnabledEditor;

	internal bool EnabledFlight;

	private GUILayoutOption[] _windowOptions;

	private readonly int _id;

	private static int _nextID = 72190852;

	private ComputerModule[] _makesActive;

	public Rect WindowPos
	{
		get
		{
			//IL_0065: Unknown result type (might be due to invalid IL or missing references)
			//IL_0033: Unknown result type (might be due to invalid IL or missing references)
			if (HighLogic.LoadedSceneIsEditor)
			{
				return new Rect(WindowVectorEditor.x, WindowVectorEditor.y, WindowVectorEditor.z, WindowVectorEditor.w);
			}
			return new Rect(WindowVector.x, WindowVector.y, WindowVector.z, WindowVector.w);
		}
		protected set
		{
			//IL_005f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0087: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d2: Unknown result type (might be due to invalid IL or missing references)
			//IL_00d7: Unknown result type (might be due to invalid IL or missing references)
			//IL_00b5: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ba: Unknown result type (might be due to invalid IL or missing references)
			//IL_00e7: Unknown result type (might be due to invalid IL or missing references)
			//IL_00e8: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ca: Unknown result type (might be due to invalid IL or missing references)
			//IL_00cb: Unknown result type (might be due to invalid IL or missing references)
			Vector4 val = default(Vector4);
			((Vector4)(ref val))._002Ector(Math.Min(Math.Max(((Rect)(ref value)).x, 0f), (float)GuiUtils.ScaledScreenWidth - ((Rect)(ref value)).width), Math.Min(Math.Max(((Rect)(ref value)).y, 0f), (float)GuiUtils.ScaledScreenHeight - ((Rect)(ref value)).height), ((Rect)(ref value)).width, ((Rect)(ref value)).height);
			val.x = Mathf.Clamp(val.x, 10f - ((Rect)(ref value)).width, (float)(GuiUtils.ScaledScreenWidth - 10));
			val.y = Mathf.Clamp(val.y, 10f - ((Rect)(ref value)).height, (float)(GuiUtils.ScaledScreenHeight - 10));
			if (!HighLogic.LoadedSceneIsEditor)
			{
				if (WindowVector != val)
				{
					Dirty = true;
					WindowVector = val;
				}
			}
			else if (WindowVectorEditor != val)
			{
				Dirty = true;
				WindowVectorEditor = val;
			}
		}
	}

	public bool ShowInFlight
	{
		get
		{
			return showInFlight;
		}
		set
		{
			if (showInFlight != value)
			{
				showInFlight = value;
				Dirty = true;
			}
		}
	}

	public bool ShowInEditor
	{
		get
		{
			return showInEditor;
		}
		set
		{
			if (showInEditor != value)
			{
				showInEditor = value;
				Dirty = true;
			}
		}
	}

	public bool IsOverlay
	{
		get
		{
			return IsOverlayConfig;
		}
		set
		{
			if (IsOverlayConfig != value)
			{
				IsOverlayConfig = value;
				Dirty = true;
			}
		}
	}

	public bool Locked
	{
		get
		{
			return LockedConfig;
		}
		set
		{
			if (LockedConfig != value)
			{
				LockedConfig = value;
				Dirty = true;
			}
		}
	}

	public bool ShowInCurrentScene
	{
		get
		{
			if (!HighLogic.LoadedSceneIsEditor)
			{
				return showInFlight;
			}
			return showInEditor;
		}
	}

	protected DisplayModule(MechJebCore core)
		: base(core)
	{
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		_id = _nextID;
		_nextID++;
	}

	protected virtual GUILayoutOption[] WindowOptions()
	{
		return Array.Empty<GUILayoutOption>();
	}

	protected void WindowGUI(int windowID, bool draggable)
	{
		//IL_0009: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00de: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f5: Unknown result type (might be due to invalid IL or missing references)
		Rect windowPos;
		if (!IsOverlayConfig)
		{
			windowPos = WindowPos;
			if (GUI.Button(new Rect(((Rect)(ref windowPos)).width - 18f, 2f, 16f, 16f), ""))
			{
				base.Enabled = false;
			}
		}
		bool flag = !LockedConfig;
		int num3;
		if (!LockedConfig && !IsOverlayConfig && Core.Settings.UseTitlebarDragging)
		{
			float num = Mouse.screenPos.x / GuiUtils.Scale;
			float num2 = Mouse.screenPos.y / GuiUtils.Scale;
			windowPos = WindowPos;
			if (num >= ((Rect)(ref windowPos)).xMin + 3f)
			{
				windowPos = WindowPos;
				float xMin = ((Rect)(ref windowPos)).xMin;
				windowPos = WindowPos;
				if (num <= xMin + ((Rect)(ref windowPos)).width - 3f)
				{
					windowPos = WindowPos;
					if (num2 >= ((Rect)(ref windowPos)).yMin + 3f)
					{
						windowPos = WindowPos;
						num3 = ((num2 <= ((Rect)(ref windowPos)).yMin + 17f) ? 1 : 0);
						goto IL_010b;
					}
				}
			}
			num3 = 0;
			goto IL_010b;
		}
		goto IL_010c;
		IL_010b:
		flag = (byte)num3 != 0;
		goto IL_010c;
		IL_010c:
		if (draggable && flag)
		{
			GUI.DragWindow();
		}
	}

	private void ProfiledWindowGUI(int windowID)
	{
		WindowGUI(windowID);
		GuiUtils.RecordTooltip(windowID);
	}

	protected virtual void WindowGUI(int windowID)
	{
		WindowGUI(windowID, draggable: true);
	}

	public virtual void DrawGUI(bool inEditor)
	{
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_0031: Unknown result type (might be due to invalid IL or missing references)
		//IL_0056: Expected O, but got Unknown
		//IL_0051: Unknown result type (might be due to invalid IL or missing references)
		if (ShowInCurrentScene)
		{
			if (_windowOptions == null)
			{
				_windowOptions = WindowOptions();
			}
			WindowPos = GUILayout.Window(_id, WindowPos, new WindowFunction(ProfiledWindowGUI), IsOverlayConfig ? "" : GetName(), _windowOptions);
			GuiUtils.ShowTooltip(_id);
		}
	}

	public override void OnSave(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnSave(local, type, global);
		if (global != null)
		{
			if (HighLogic.LoadedSceneIsEditor)
			{
				EnabledEditor = base.Enabled;
			}
			if (HighLogic.LoadedSceneIsFlight)
			{
				EnabledFlight = base.Enabled;
			}
			global.AddValue("enabledEditor", EnabledEditor);
			global.AddValue("enabledFlight", EnabledFlight);
		}
	}

	public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnLoad(local, type, global);
		bool flag = true;
		if (global != null && global.HasValue("enabledEditor") && bool.TryParse(global.GetValue("enabledEditor"), out var result))
		{
			EnabledEditor = result;
			flag = false;
			if (HighLogic.LoadedSceneIsEditor)
			{
				base.Enabled = result;
			}
		}
		if (global != null && global.HasValue("enabledFlight") && bool.TryParse(global.GetValue("enabledFlight"), out var result2))
		{
			EnabledFlight = result2;
			flag = false;
			if (HighLogic.LoadedSceneIsFlight)
			{
				base.Enabled = result2;
			}
		}
		if (flag)
		{
			if (global != null && global.HasValue("enabled") && bool.TryParse(global.GetValue("enabled"), out var result3))
			{
				base.Enabled = result3;
			}
			EnabledEditor = base.Enabled;
			EnabledFlight = base.Enabled;
		}
	}

	public virtual bool IsActive()
	{
		if (_makesActive == null)
		{
			_makesActive = new ComputerModule[6] { Core.Attitude, Core.Thrust, Core.Rover, Core.Node, Core.RCS, Core.Rcsbal };
		}
		bool flag = false;
		for (int i = 0; i < _makesActive.Length; i++)
		{
			ComputerModule computerModule = _makesActive[i];
			if (computerModule != null && (flag |= computerModule.Users.RecursiveUser(this)))
			{
				break;
			}
		}
		return flag;
	}

	public override void UnlockCheck()
	{
		if (UnlockChecked)
		{
			return;
		}
		bool enabled = base.Enabled;
		base.Enabled = true;
		base.UnlockCheck();
		if (unlockParts.Trim().Length > 0 || unlockTechs.Trim().Length > 0 || !IsSpaceCenterUpgradeUnlocked())
		{
			Hidden = !base.Enabled;
			if (Hidden)
			{
				enabled = false;
			}
		}
		base.Enabled = enabled;
	}

	public virtual string GetName()
	{
		return "Display Module";
	}

	public virtual string IconName()
	{
		return "Display Module Icon";
	}
}
