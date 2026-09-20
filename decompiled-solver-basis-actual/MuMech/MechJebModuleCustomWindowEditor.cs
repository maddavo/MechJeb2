using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleCustomWindowEditor : DisplayModule
{
	public readonly List<InfoItem> registry = new List<InfoItem>();

	public MechJebModuleCustomInfoWindow editedWindow;

	private int selectedItemIndex = -1;

	[Persistent(pass = 4)]
	public InfoItem.Category itemCategory;

	private const int CUSTOM_WINDOWS_VERSION = 2;

	[Persistent(pass = 4)]
	public int CustomWindowsVersion = -1;

	private static readonly string[] categories = Enum.GetNames(typeof(InfoItem.Category));

	private int presetIndex;

	private bool editingBackground;

	private bool editingText;

	private readonly Stopwatch _valueInfoItemStopwatch = new Stopwatch();

	private readonly Stopwatch _actionInfoItemStopwatch = new Stopwatch();

	private readonly Stopwatch _toggleInfoItemStopwatch = new Stopwatch();

	private readonly Stopwatch _generalInfoItemStopwatch = new Stopwatch();

	private readonly Stopwatch _editableInfoItemStopwatch = new Stopwatch();

	private static readonly Dictionary<Type, List<Tuple<MemberInfo, Attribute>>> _cache = new Dictionary<Type, List<Tuple<MemberInfo, Attribute>>>();

	private Vector2 scrollPos;

	private Vector2 scrollPos2;

	public bool RegenerateDefaultWindows { get; private set; }

	public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnLoad(local, type, global);
		registry.Clear();
		editedWindow = null;
		RegenerateDefaultWindows = false;
		_valueInfoItemStopwatch.Reset();
		_actionInfoItemStopwatch.Reset();
		_toggleInfoItemStopwatch.Reset();
		_generalInfoItemStopwatch.Reset();
		_editableInfoItemStopwatch.Reset();
		Stopwatch stopwatch = new Stopwatch();
		stopwatch.Start();
		RegisterInfoItems(base.VesselState);
		foreach (ComputerModule computerModule in Core.GetComputerModules<ComputerModule>())
		{
			RegisterInfoItems(computerModule);
		}
		stopwatch.Stop();
		ComputerModule.Print($"Registered {registry.Count} info items:  value:{_valueInfoItemStopwatch.ElapsedMilliseconds} ms action:{_actionInfoItemStopwatch.ElapsedMilliseconds} ms  toggle:{_toggleInfoItemStopwatch.ElapsedMilliseconds} ms  general:{_generalInfoItemStopwatch.ElapsedMilliseconds} ms  editable:{_editableInfoItemStopwatch.ElapsedMilliseconds} ms  total:{stopwatch.ElapsedMilliseconds} ms");
		if (global == null)
		{
			return;
		}
		if (!global.HasValue("CustomWindowsVersion") || CustomWindowsVersion < 2)
		{
			CustomWindowsVersion = 2;
			RegenerateDefaultWindows = true;
			return;
		}
		ConfigNode[] nodes = global.GetNodes(typeof(MechJebModuleCustomInfoWindow).Name);
		foreach (ConfigNode val in nodes)
		{
			MechJebModuleCustomInfoWindow mechJebModuleCustomInfoWindow = new MechJebModuleCustomInfoWindow(Core);
			ConfigNode.LoadObjectFromConfig((object)mechJebModuleCustomInfoWindow, val);
			mechJebModuleCustomInfoWindow.UpdateRefreshRate();
			bool flag = true;
			if (val.HasValue("enabledEditor") && bool.TryParse(val.GetValue("enabledEditor"), out var result))
			{
				mechJebModuleCustomInfoWindow.EnabledEditor = result;
				flag = false;
				if (HighLogic.LoadedSceneIsEditor)
				{
					mechJebModuleCustomInfoWindow.Enabled = result;
				}
			}
			if (val.HasValue("enabledFlight") && bool.TryParse(val.GetValue("enabledFlight"), out var result2))
			{
				mechJebModuleCustomInfoWindow.EnabledFlight = result2;
				flag = false;
				if (HighLogic.LoadedSceneIsFlight)
				{
					mechJebModuleCustomInfoWindow.Enabled = result2;
				}
			}
			if (flag)
			{
				if (val.HasValue("enabled") && bool.TryParse(val.GetValue("enabled"), out var result3))
				{
					mechJebModuleCustomInfoWindow.Enabled = result3;
					mechJebModuleCustomInfoWindow.EnabledEditor = mechJebModuleCustomInfoWindow.Enabled;
					mechJebModuleCustomInfoWindow.EnabledFlight = mechJebModuleCustomInfoWindow.Enabled;
				}
				mechJebModuleCustomInfoWindow.EnabledEditor = mechJebModuleCustomInfoWindow.Enabled;
				mechJebModuleCustomInfoWindow.EnabledFlight = mechJebModuleCustomInfoWindow.Enabled;
			}
			mechJebModuleCustomInfoWindow.items = new List<InfoItem>();
			if (val.HasNode("items"))
			{
				ConfigNode[] nodes2 = val.GetNode("items").GetNodes("InfoItem");
				foreach (ConfigNode val2 in nodes2)
				{
					string id = val2.GetValue("id");
					InfoItem infoItem = registry.FirstOrDefault((InfoItem item) => item.id == id);
					if (infoItem != null)
					{
						mechJebModuleCustomInfoWindow.items.Add(infoItem);
					}
				}
			}
			Core.AddComputerModuleLater(mechJebModuleCustomInfoWindow);
		}
	}

	public override void OnSave(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnSave(local, type, global);
		if (global == null)
		{
			return;
		}
		foreach (MechJebModuleCustomInfoWindow computerModule in Core.GetComputerModules<MechJebModuleCustomInfoWindow>())
		{
			string name = typeof(MechJebModuleCustomInfoWindow).Name;
			ConfigNode obj = ConfigNode.CreateConfigFromObject((object)computerModule, 4, (ConfigNode)null);
			if (HighLogic.LoadedSceneIsEditor)
			{
				computerModule.EnabledEditor = computerModule.Enabled;
			}
			if (HighLogic.LoadedSceneIsFlight)
			{
				computerModule.EnabledFlight = computerModule.Enabled;
			}
			obj.AddValue("enabledFlight", computerModule.EnabledFlight);
			obj.AddValue("enabledEditor", computerModule.EnabledEditor);
			obj.CopyTo(global.AddNode(name));
			computerModule.Dirty = false;
		}
	}

	public override void OnStart(StartState state)
	{
		editedWindow = Core.GetComputerModule<MechJebModuleCustomInfoWindow>();
	}

	private void RegisterInfoItems(object obj)
	{
		Type type = obj.GetType();
		if (!_cache.ContainsKey(type))
		{
			_cache.Add(type, new List<Tuple<MemberInfo, Attribute>>());
			MemberInfo[] members = type.GetMembers(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.FlattenHierarchy | BindingFlags.GetField | BindingFlags.GetProperty);
			foreach (MemberInfo memberInfo in members)
			{
				object[] customAttributes = memberInfo.GetCustomAttributes(inherit: true);
				for (int j = 0; j < customAttributes.Length; j++)
				{
					Attribute attribute = (Attribute)customAttributes[j];
					if (!(attribute is ValueInfoItemAttribute))
					{
						if (!(attribute is ActionInfoItemAttribute))
						{
							if (!(attribute is ToggleInfoItemAttribute))
							{
								if (!(attribute is GeneralInfoItemAttribute))
								{
									if (attribute is EditableInfoItemAttribute)
									{
										_cache[type].Add(new Tuple<MemberInfo, Attribute>(memberInfo, attribute));
									}
								}
								else
								{
									_cache[type].Add(new Tuple<MemberInfo, Attribute>(memberInfo, attribute));
								}
							}
							else
							{
								_cache[type].Add(new Tuple<MemberInfo, Attribute>(memberInfo, attribute));
							}
						}
						else
						{
							_cache[type].Add(new Tuple<MemberInfo, Attribute>(memberInfo, attribute));
						}
					}
					else
					{
						_cache[type].Add(new Tuple<MemberInfo, Attribute>(memberInfo, attribute));
					}
				}
			}
		}
		foreach (var (memberInfo3, attribute3) in _cache[type])
		{
			if (!(attribute3 is ValueInfoItemAttribute attribute4))
			{
				if (!(attribute3 is ActionInfoItemAttribute attribute5))
				{
					if (!(attribute3 is ToggleInfoItemAttribute attribute6))
					{
						if (!(attribute3 is GeneralInfoItemAttribute attribute7))
						{
							if (attribute3 is EditableInfoItemAttribute attribute8)
							{
								_editableInfoItemStopwatch.Start();
								registry.Add(new EditableInfoItem(obj, memberInfo3, attribute8));
								_editableInfoItemStopwatch.Stop();
							}
						}
						else
						{
							_generalInfoItemStopwatch.Start();
							registry.Add(new GeneralInfoItem(obj, (MethodInfo)memberInfo3, attribute7));
							_generalInfoItemStopwatch.Stop();
						}
					}
					else
					{
						_toggleInfoItemStopwatch.Start();
						registry.Add(new ToggleInfoItem(obj, memberInfo3, attribute6));
						_toggleInfoItemStopwatch.Stop();
					}
				}
				else
				{
					_actionInfoItemStopwatch.Start();
					registry.Add(new ActionInfoItem(obj, (MethodInfo)memberInfo3, attribute5));
					_actionInfoItemStopwatch.Stop();
				}
			}
			else
			{
				_valueInfoItemStopwatch.Start();
				registry.Add(new ValueInfoItem(obj, memberInfo3, attribute4));
				_valueInfoItemStopwatch.Stop();
			}
		}
	}

	private void AddNewWindow()
	{
		editedWindow = new MechJebModuleCustomInfoWindow(Core);
		if (HighLogic.LoadedSceneIsEditor)
		{
			editedWindow.ShowInEditor = true;
		}
		if (HighLogic.LoadedSceneIsFlight)
		{
			editedWindow.ShowInFlight = true;
		}
		Core.AddComputerModule(editedWindow);
		editedWindow.Enabled = true;
		editedWindow.Dirty = true;
	}

	private void RemoveCurrentWindow()
	{
		if (editedWindow != null)
		{
			Core.RemoveComputerModule(editedWindow);
			editedWindow = Core.GetComputerModule<MechJebModuleCustomInfoWindow>();
		}
	}

	public override void DrawGUI(bool inEditor)
	{
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_0073: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_010b: Unknown result type (might be due to invalid IL or missing references)
		//IL_010c: Unknown result type (might be due to invalid IL or missing references)
		base.DrawGUI(inEditor);
		Rect windowPos;
		if (editingBackground && editedWindow != null)
		{
			editedWindow.Init();
			windowPos = base.WindowPos;
			int positionLeft = (int)((Rect)(ref windowPos)).xMax + 5;
			windowPos = base.WindowPos;
			Color val = ColorPickerRGB.DrawGUI(positionLeft, (int)((Rect)(ref windowPos)).yMin, editedWindow.backgroundColor);
			if (editedWindow.backgroundColor != val)
			{
				editedWindow.backgroundColor = val;
				editedWindow.background.SetPixel(0, 0, editedWindow.backgroundColor);
				editedWindow.background.Apply();
				editedWindow.Dirty = true;
			}
		}
		if (editingText && editedWindow != null)
		{
			windowPos = base.WindowPos;
			int positionLeft2 = (int)((Rect)(ref windowPos)).xMax + 5;
			windowPos = base.WindowPos;
			Color val2 = ColorPickerRGB.DrawGUI(positionLeft2, (int)((Rect)(ref windowPos)).yMin, editedWindow.text);
			if (editedWindow.text != val2)
			{
				editedWindow.text = val2;
				editedWindow.Dirty = true;
			}
		}
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_02ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02fb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0516: Unknown result type (might be due to invalid IL or missing references)
		//IL_0520: Unknown result type (might be due to invalid IL or missing references)
		//IL_0525: Unknown result type (might be due to invalid IL or missing references)
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		if (editedWindow == null)
		{
			editedWindow = Core.GetComputerModule<MechJebModuleCustomInfoWindow>();
		}
		if (editedWindow == null)
		{
			if (GUILayout.Button(Localizer.Format("#MechJeb_WindowEd_button1"), Array.Empty<GUILayoutOption>()))
			{
				AddNewWindow();
			}
		}
		else
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			if (GUILayout.Button(Localizer.Format("#MechJeb_WindowEd_button1"), Array.Empty<GUILayoutOption>()))
			{
				AddNewWindow();
			}
			if (GUILayout.Button(Localizer.Format("#MechJeb_WindowEd_button2"), Array.Empty<GUILayoutOption>()))
			{
				RemoveCurrentWindow();
			}
			GUILayout.EndHorizontal();
		}
		if (editedWindow != null)
		{
			List<ComputerModule> computerModules = Core.GetComputerModules<MechJebModuleCustomInfoWindow>();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_WindowEd_Edtitle"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			int index = computerModules.IndexOf(editedWindow);
			index = GuiUtils.ArrowSelector(index, computerModules.Count, delegate
			{
				string text = GUILayout.TextField(editedWindow.title, (GUILayoutOption[])(object)new GUILayoutOption[2]
				{
					GuiUtils.LayoutWidth(120f),
					GuiUtils.LayoutNoExpandWidth
				});
				if (editedWindow.title != text)
				{
					editedWindow.title = text;
					editedWindow.Dirty = true;
				}
			});
			editedWindow = (MechJebModuleCustomInfoWindow)computerModules[index];
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_WindowEd_label1"), Array.Empty<GUILayoutOption>());
			editedWindow.ShowInFlight = GUILayout.Toggle(editedWindow.ShowInFlight, Localizer.Format("#MechJeb_WindowEd_checkbox1"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(60f) });
			editedWindow.ShowInEditor = GUILayout.Toggle(editedWindow.ShowInEditor, Localizer.Format("#MechJeb_WindowEd_checkbox2"), Array.Empty<GUILayoutOption>());
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			editedWindow.IsOverlay = GUILayout.Toggle(editedWindow.IsOverlay, Localizer.Format("#MechJeb_WindowEd_checkbox3"), Array.Empty<GUILayoutOption>());
			editedWindow.Locked = GUILayout.Toggle(editedWindow.Locked, Localizer.Format("#MechJeb_WindowEd_checkbox4"), Array.Empty<GUILayoutOption>());
			editedWindow.IsCompact = GUILayout.Toggle(editedWindow.IsCompact, Localizer.Format("#MechJeb_WindowEd_checkbox5"), Array.Empty<GUILayoutOption>());
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_WindowEd_label2"), Array.Empty<GUILayoutOption>());
			bool flag = editingText;
			editingText = GUILayout.Toggle(editingText, Localizer.Format("#MechJeb_WindowEd_checkbox6"), Array.Empty<GUILayoutOption>());
			if (editingText && editingText != flag)
			{
				editingBackground = false;
			}
			flag = editingBackground;
			editingBackground = GUILayout.Toggle(editingBackground, Localizer.Format("#MechJeb_WindowEd_checkbox7"), Array.Empty<GUILayoutOption>());
			if (editingBackground && editingBackground != flag)
			{
				editingText = false;
			}
			GUILayout.EndHorizontal();
			GUILayout.Label(Localizer.Format("#MechJeb_WindowEd_label3"), Array.Empty<GUILayoutOption>());
			GUILayout.BeginVertical((GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.Height(100f) });
			scrollPos = GUILayout.BeginScrollView(scrollPos, Array.Empty<GUILayoutOption>());
			for (int i = 0; i < editedWindow.items.Count; i++)
			{
				GUIStyle val = ((i == selectedItemIndex) ? GuiUtils.YellowLabel : GUI.skin.label);
				if (GUILayout.Button(Localizer.Format(editedWindow.items[i].description), val, Array.Empty<GUILayoutOption>()))
				{
					selectedItemIndex = i;
				}
			}
			GUILayout.EndScrollView();
			GUILayout.EndVertical();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			if (selectedItemIndex < 0 || selectedItemIndex >= editedWindow.items.Count)
			{
				selectedItemIndex = -1;
			}
			if (GUILayout.Button(Localizer.Format("#MechJeb_WindowEd_button3"), Array.Empty<GUILayoutOption>()) && selectedItemIndex != -1)
			{
				editedWindow.items.RemoveAt(selectedItemIndex);
			}
			if (GUILayout.Button(Localizer.Format("#MechJeb_WindowEd_button4"), Array.Empty<GUILayoutOption>()) && selectedItemIndex != -1 && selectedItemIndex > 0)
			{
				InfoItem item = editedWindow.items[selectedItemIndex];
				editedWindow.items.RemoveAt(selectedItemIndex);
				editedWindow.items.Insert(selectedItemIndex - 1, item);
				selectedItemIndex--;
			}
			if (GUILayout.Button(Localizer.Format("#MechJeb_WindowEd_button5"), Array.Empty<GUILayoutOption>()) && selectedItemIndex != -1 && selectedItemIndex < editedWindow.items.Count)
			{
				InfoItem item2 = editedWindow.items[selectedItemIndex];
				editedWindow.items.RemoveAt(selectedItemIndex);
				editedWindow.items.Insert(selectedItemIndex + 1, item2);
				selectedItemIndex++;
			}
			GUILayout.EndHorizontal();
			GUILayout.Label(Localizer.Format("#MechJeb_WindowEd_label4"), Array.Empty<GUILayoutOption>());
			itemCategory = (InfoItem.Category)GuiUtils.ComboBox.Box((int)itemCategory, categories, this);
			scrollPos2 = GUILayout.BeginScrollView(scrollPos2, Array.Empty<GUILayoutOption>());
			foreach (InfoItem item3 in from it in registry
				where it.category == itemCategory
				orderby it.description
				select it)
			{
				if (GUILayout.Button(Localizer.Format(item3.description), GuiUtils.YellowOnHover, Array.Empty<GUILayoutOption>()))
				{
					editedWindow.items.Add(item3);
				}
			}
			GUILayout.EndScrollView();
		}
		GUILayout.Label(Localizer.Format("#MechJeb_WindowEd_label5"), GuiUtils.MiddleCenterLabel, Array.Empty<GUILayoutOption>());
		presetIndex = GuiUtils.ArrowSelector(presetIndex, CustomWindowPresets.presets.Length, delegate
		{
			if (GUILayout.Button(CustomWindowPresets.presets[presetIndex].name, Array.Empty<GUILayoutOption>()))
			{
				MechJebModuleCustomInfoWindow mechJebModuleCustomInfoWindow = CreateWindowFromSharingString(CustomWindowPresets.presets[presetIndex].sharingString);
				if (mechJebModuleCustomInfoWindow != null)
				{
					editedWindow = mechJebModuleCustomInfoWindow;
				}
			}
		});
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(200f),
			GUILayout.Height(540f)
		};
	}

	public override string GetName()
	{
		return CachedLocalizer.Instance.MechJebWindowEdTitle;
	}

	public override string IconName()
	{
		return "Custom Window Editor";
	}

	public MechJebModuleCustomWindowEditor(MechJebCore core)
		: base(core)
	{
		base.ShowInFlight = true;
		base.ShowInEditor = true;
	}

	public void AddDefaultWindows()
	{
		CreateWindowFromSharingString(CustomWindowPresets.presets[0].sharingString).Enabled = false;
		CreateWindowFromSharingString(CustomWindowPresets.presets[1].sharingString).Enabled = false;
		CreateWindowFromSharingString(CustomWindowPresets.presets[2].sharingString).Enabled = false;
		CreateWindowFromSharingString(CustomWindowPresets.presets[3].sharingString).Enabled = false;
		CreateWindowFromSharingString(CustomWindowPresets.presets[4].sharingString).Enabled = false;
		CreateWindowFromSharingString(CustomWindowPresets.presets[5].sharingString).Enabled = false;
		CreateWindowFromSharingString(CustomWindowPresets.presets[6].sharingString).Enabled = false;
		CreateWindowFromSharingString(CustomWindowPresets.presets[7].sharingString).Enabled = false;
		CreateWindowFromSharingString(CustomWindowPresets.presets[10].sharingString).Enabled = false;
		CreateWindowFromSharingString(CustomWindowPresets.presets[12].sharingString).Enabled = false;
	}

	public MechJebModuleCustomInfoWindow CreateWindowFromSharingString(string sharingString)
	{
		string[] array = sharingString.Split(new char[2] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
		if (array[0] != "--- MechJeb Custom Window ---")
		{
			ScreenMessages.PostScreenMessage(Localizer.Format("#MechJeb_WindowEd_CustomInfoWindow_Scrmsg2"), 3f, (ScreenMessageStyle)2);
			return null;
		}
		MechJebModuleCustomInfoWindow mechJebModuleCustomInfoWindow = new MechJebModuleCustomInfoWindow(Core);
		Core.AddComputerModule(mechJebModuleCustomInfoWindow);
		mechJebModuleCustomInfoWindow.Enabled = true;
		mechJebModuleCustomInfoWindow.FromSharingString(array, registry);
		return mechJebModuleCustomInfoWindow;
	}
}
