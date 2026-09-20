using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace MuMech;

public static class GuiUtils
{
	public enum SkinType
	{
		DEFAULT,
		MECH_JEB1,
		COMPACT
	}

	public class ComboBox
	{
		[Serializable]
		[CompilerGenerated]
		private sealed class _003C_003Ec
		{
			public static readonly _003C_003Ec _003C_003E9 = new _003C_003Ec();

			public static WindowFunction _003C_003E9__12_0;

			internal void _003CDrawGUI_003Eb__12_0(int identifier)
			{
				_selectedItem = GUILayout.SelectionGrid(-1, _entries, 1, YellowOnHover, Array.Empty<GUILayoutOption>());
				if (GUI.changed)
				{
					_popupActive = false;
				}
			}
		}

		private static Rect _rect;

		private static object _popupOwner;

		private static string[] _entries;

		private static bool _popupActive;

		private static int _selectedItem;

		private static readonly int _id;

		private static readonly GUIStyle _style;

		public static bool IsPopupActive
		{
			get
			{
				if (_popupOwner != null && ((Rect)(ref _rect)).height > 0f)
				{
					return _popupActive;
				}
				return false;
			}
		}

		public static Rect PopupRect => _rect;

		static ComboBox()
		{
			//IL_0015: Unknown result type (might be due to invalid IL or missing references)
			//IL_001a: Unknown result type (might be due to invalid IL or missing references)
			//IL_0026: Unknown result type (might be due to invalid IL or missing references)
			//IL_0037: Expected O, but got Unknown
			_id = GUIUtility.GetControlID((FocusType)2);
			GUIStyle val = new GUIStyle(GUI.skin.window);
			val.normal.background = null;
			val.onNormal.background = null;
			_style = val;
			_style.border.top = _style.border.bottom;
			_style.padding.top = _style.padding.bottom;
		}

		public static void DrawGUI()
		{
			//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ff: Unknown result type (might be due to invalid IL or missing references)
			//IL_0104: Unknown result type (might be due to invalid IL or missing references)
			//IL_010e: Unknown result type (might be due to invalid IL or missing references)
			//IL_00e5: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
			//IL_00f0: Expected O, but got Unknown
			//IL_011f: Unknown result type (might be due to invalid IL or missing references)
			if (_popupOwner == null || ((Rect)(ref _rect)).height == 0f || !_popupActive)
			{
				return;
			}
			if ((Object)(object)_style.normal.background == (Object)null)
			{
				_style.normal.background = MechJebBundlesManager.comboBoxBackground;
				_style.onNormal.background = MechJebBundlesManager.comboBoxBackground;
			}
			((Rect)(ref _rect)).x = Math.Max(0f, Math.Min(((Rect)(ref _rect)).x, (float)ScaledScreenWidth - ((Rect)(ref _rect)).width));
			((Rect)(ref _rect)).y = Math.Max(0f, Math.Min(((Rect)(ref _rect)).y, (float)ScaledScreenHeight - ((Rect)(ref _rect)).height));
			int id = _id;
			Rect rect = _rect;
			object obj = _003C_003Ec._003C_003E9__12_0;
			if (obj == null)
			{
				WindowFunction val = delegate
				{
					_selectedItem = GUILayout.SelectionGrid(-1, _entries, 1, YellowOnHover, Array.Empty<GUILayoutOption>());
					if (GUI.changed)
					{
						_popupActive = false;
					}
				};
				_003C_003Ec._003C_003E9__12_0 = val;
				obj = (object)val;
			}
			_rect = GUILayout.Window(id, rect, (WindowFunction)obj, "", _style, Array.Empty<GUILayoutOption>());
			if ((int)Event.current.type == 0 && !((Rect)(ref _rect)).Contains(Event.current.mousePosition))
			{
				_popupOwner = null;
			}
		}

		public static int Box(int selectedItem, string[] entries, object caller, bool expandWidth = true)
		{
			//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
			//IL_00ce: Invalid comparison between Unknown and I4
			//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
			//IL_00be: Unknown result type (might be due to invalid IL or missing references)
			//IL_00f2: Unknown result type (might be due to invalid IL or missing references)
			//IL_00f7: Unknown result type (might be due to invalid IL or missing references)
			//IL_00fc: Unknown result type (might be due to invalid IL or missing references)
			//IL_0101: Unknown result type (might be due to invalid IL or missing references)
			//IL_0106: Unknown result type (might be due to invalid IL or missing references)
			//IL_010f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0120: Unknown result type (might be due to invalid IL or missing references)
			//IL_0125: Unknown result type (might be due to invalid IL or missing references)
			//IL_0135: Unknown result type (might be due to invalid IL or missing references)
			//IL_0142: Unknown result type (might be due to invalid IL or missing references)
			//IL_015d: Unknown result type (might be due to invalid IL or missing references)
			//IL_016a: Unknown result type (might be due to invalid IL or missing references)
			if (entries.Length == 0)
			{
				return 0;
			}
			if (entries.Length == 1)
			{
				GUILayout.Label(entries[0], Array.Empty<GUILayoutOption>());
				return 0;
			}
			if (selectedItem >= entries.Length)
			{
				selectedItem = entries.Length - 1;
			}
			if (DontUseDropDownMenu)
			{
				return ArrowSelector(selectedItem, entries.Length, entries[selectedItem], expandWidth);
			}
			if (_popupOwner == caller && !_popupActive)
			{
				_popupOwner = null;
				selectedItem = _selectedItem;
				GUI.changed = true;
			}
			bool changed = GUI.changed;
			if (GUILayout.Button("↓ " + entries[selectedItem] + " ↓", (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.ExpandWidth(expandWidth) }))
			{
				GUI.changed = changed;
				_popupOwner = caller;
				_popupActive = true;
				_entries = entries;
				_rect = new Rect(0f, 0f, 0f, 0f);
			}
			if ((int)Event.current.type == 7 && _popupOwner == caller && ((Rect)(ref _rect)).height == 0f)
			{
				_rect = GUILayoutUtility.GetLastRect();
				Vector2 val = Vector2.op_Implicit(Input.mousePosition);
				val.y = (float)Screen.height - val.y;
				Vector2 mousePosition = Event.current.mousePosition;
				((Rect)(ref _rect)).x = (((Rect)(ref _rect)).x + val.x) / Scale - mousePosition.x;
				((Rect)(ref _rect)).y = (((Rect)(ref _rect)).y + val.y) / Scale - mousePosition.y;
			}
			return selectedItem;
		}
	}

	[Serializable]
	[CompilerGenerated]
	private sealed class _003C_003Ec
	{
		public static readonly _003C_003Ec _003C_003E9 = new _003C_003Ec();

		public static WindowFunction _003C_003E9__94_0;

		internal void _003CShowTooltip_003Eb__94_0(int _)
		{
		}
	}

	private static GUIStyle _yellowOnHover;

	private static GUIStyle _yellowLabel;

	private static GUIStyle _redLabel;

	private static GUIStyle _greenLabel;

	private static GUIStyle _orangeLabel;

	private static GUIStyle _middleCenterLabel;

	private static GUIStyle _middleRightLabel;

	private static GUIStyle _upperCenterLabel;

	private static GUIStyle _labelNoWrap;

	private static GUIStyle _greenToggle;

	private static GUIStyle _redToggle;

	private static GUIStyle _yellowToggle;

	public static GUISkin Skin;

	public static float Scale = 1f;

	public static int ScaledScreenWidth = 1;

	public static int ScaledScreenHeight = 1;

	public static bool DontUseDropDownMenu = false;

	public static bool ShowAdvancedWindowSettings = false;

	public static GUISkin DefaultSkin;

	public static GUISkin CompactSkin;

	public static GUISkin TransparentSkin;

	private static GUILayoutOption _layoutExpandWidth;

	private static GUILayoutOption _layoutNoExpandWidth;

	private static readonly Dictionary<float, GUILayoutOption> _layoutWidthDict = new Dictionary<float, GUILayoutOption>(16);

	private static GUIStyle _arrowSelectorStyeGuiStyleExpand;

	private static GUIStyle _arrowSelectorStyeGuiStyleNoExpand;

	private static readonly Dictionary<int, string> _tooltipTexts = new Dictionary<int, string>();

	private static GUIStyle _tooltipStyle;

	private static Rect _tooltipRect;

	private static DateTime _tooltipBeginDt;

	private static bool _tooltipChanged;

	private const float TooltipMaxWidth = 200f;

	private const double TooltipShowDelay = 1000.0;

	private static readonly int _tooltipWindowId = "MechJebTooltip".GetHashCode();

	public static GUIStyle YellowOnHover
	{
		get
		{
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0022: Unknown result type (might be due to invalid IL or missing references)
			//IL_0031: Expected O, but got Unknown
			//IL_0033: Unknown result type (might be due to invalid IL or missing references)
			//IL_0039: Expected O, but got Unknown
			//IL_0050: Unknown result type (might be due to invalid IL or missing references)
			if (_yellowOnHover != null)
			{
				return _yellowOnHover;
			}
			GUIStyle val = new GUIStyle(GUI.skin.label);
			val.hover.textColor = Color.yellow;
			_yellowOnHover = val;
			Texture2D val2 = new Texture2D(1, 1);
			val2.SetPixel(0, 0, new Color(0f, 0f, 0f, 0f));
			val2.Apply();
			_yellowOnHover.hover.background = val2;
			return _yellowOnHover;
		}
	}

	public static GUIStyle YellowLabel
	{
		get
		{
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0022: Unknown result type (might be due to invalid IL or missing references)
			//IL_002c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0032: Unknown result type (might be due to invalid IL or missing references)
			//IL_0041: Expected O, but got Unknown
			if (_yellowLabel != null)
			{
				return _yellowLabel;
			}
			GUIStyle val = new GUIStyle(GUI.skin.label);
			val.normal.textColor = Color.yellow;
			val.hover.textColor = Color.yellow;
			_yellowLabel = val;
			return _yellowLabel;
		}
	}

	public static GUIStyle RedLabel
	{
		get
		{
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0022: Unknown result type (might be due to invalid IL or missing references)
			//IL_002c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0032: Unknown result type (might be due to invalid IL or missing references)
			//IL_0041: Expected O, but got Unknown
			if (_redLabel != null)
			{
				return _redLabel;
			}
			GUIStyle val = new GUIStyle(GUI.skin.label);
			val.normal.textColor = Color.red;
			val.hover.textColor = Color.red;
			_redLabel = val;
			return _redLabel;
		}
	}

	public static GUIStyle GreenLabel
	{
		get
		{
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0022: Unknown result type (might be due to invalid IL or missing references)
			//IL_002c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0032: Unknown result type (might be due to invalid IL or missing references)
			//IL_0041: Expected O, but got Unknown
			if (_greenLabel != null)
			{
				return _greenLabel;
			}
			GUIStyle val = new GUIStyle(GUI.skin.label);
			val.normal.textColor = Color.green;
			val.hover.textColor = Color.green;
			_greenLabel = val;
			return _greenLabel;
		}
	}

	public static GUIStyle OrangeLabel
	{
		get
		{
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0022: Unknown result type (might be due to invalid IL or missing references)
			//IL_002c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0032: Unknown result type (might be due to invalid IL or missing references)
			//IL_0041: Expected O, but got Unknown
			if (_orangeLabel != null)
			{
				return _orangeLabel;
			}
			GUIStyle val = new GUIStyle(GUI.skin.label);
			val.normal.textColor = Color.green;
			val.hover.textColor = Color.green;
			_orangeLabel = val;
			return _orangeLabel;
		}
	}

	public static GUIStyle MiddleCenterLabel
	{
		get
		{
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0028: Expected O, but got Unknown
			if (_middleCenterLabel != null)
			{
				return _middleCenterLabel;
			}
			_middleCenterLabel = new GUIStyle(GUI.skin.label)
			{
				alignment = (TextAnchor)4
			};
			return _middleCenterLabel;
		}
	}

	public static GUIStyle MiddleRightLabel
	{
		get
		{
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0028: Expected O, but got Unknown
			if (_middleRightLabel != null)
			{
				return _middleRightLabel;
			}
			_middleRightLabel = new GUIStyle(GUI.skin.label)
			{
				alignment = (TextAnchor)5
			};
			return _middleRightLabel;
		}
	}

	public static GUIStyle UpperCenterLabel
	{
		get
		{
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0028: Expected O, but got Unknown
			if (_upperCenterLabel != null)
			{
				return _upperCenterLabel;
			}
			_upperCenterLabel = new GUIStyle(GUI.skin.label)
			{
				alignment = (TextAnchor)1
			};
			return _upperCenterLabel;
		}
	}

	public static GUIStyle LabelNoWrap
	{
		get
		{
			//IL_0013: Unknown result type (might be due to invalid IL or missing references)
			//IL_0018: Unknown result type (might be due to invalid IL or missing references)
			//IL_001f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0025: Expected O, but got Unknown
			object obj = _labelNoWrap;
			if (obj == null)
			{
				GUIStyle val = new GUIStyle(GUI.skin.label)
				{
					wordWrap = false
				};
				_labelNoWrap = val;
				obj = (object)val;
			}
			return (GUIStyle)obj;
		}
	}

	public static GUIStyle GreenToggle
	{
		get
		{
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0022: Unknown result type (might be due to invalid IL or missing references)
			//IL_002c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0032: Unknown result type (might be due to invalid IL or missing references)
			//IL_0041: Expected O, but got Unknown
			if (_greenToggle != null)
			{
				return _greenToggle;
			}
			GUIStyle val = new GUIStyle(GUI.skin.toggle);
			val.onHover.textColor = Color.green;
			val.onNormal.textColor = Color.green;
			_greenToggle = val;
			return _greenToggle;
		}
	}

	public static GUIStyle RedToggle
	{
		get
		{
			//IL_0017: Unknown result type (might be due to invalid IL or missing references)
			//IL_001c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0022: Unknown result type (might be due to invalid IL or missing references)
			//IL_002c: Unknown result type (might be due to invalid IL or missing references)
			//IL_0032: Unknown result type (might be due to invalid IL or missing references)
			//IL_0041: Expected O, but got Unknown
			if (_redToggle != null)
			{
				return _redToggle;
			}
			GUIStyle val = new GUIStyle(GUI.skin.toggle);
			val.onHover.textColor = Color.red;
			val.onNormal.textColor = Color.red;
			_redToggle = val;
			return _redToggle;
		}
	}

	public static GUIStyle YellowToggle
	{
		get
		{
			//IL_0013: Unknown result type (might be due to invalid IL or missing references)
			//IL_0018: Unknown result type (might be due to invalid IL or missing references)
			//IL_001e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0028: Unknown result type (might be due to invalid IL or missing references)
			//IL_002e: Unknown result type (might be due to invalid IL or missing references)
			//IL_0038: Unknown result type (might be due to invalid IL or missing references)
			//IL_003e: Expected O, but got Unknown
			object obj = _yellowToggle;
			if (obj == null)
			{
				GUIStyle val = new GUIStyle(GUI.skin.toggle);
				val.onHover.textColor = Color.yellow;
				val.onNormal.textColor = Color.yellow;
				_yellowToggle = val;
				obj = (object)val;
			}
			return (GUIStyle)obj;
		}
	}

	public static GUILayoutOption LayoutExpandWidth => _layoutExpandWidth ?? (_layoutExpandWidth = GUILayout.ExpandWidth(true));

	public static GUILayoutOption LayoutNoExpandWidth => _layoutNoExpandWidth ?? (_layoutNoExpandWidth = GUILayout.ExpandWidth(false));

	public static GUIStyle ArrowSelectorStyeGuiStyleExpand
	{
		get
		{
			//IL_0013: Unknown result type (might be due to invalid IL or missing references)
			//IL_0018: Unknown result type (might be due to invalid IL or missing references)
			//IL_001f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0026: Unknown result type (might be due to invalid IL or missing references)
			//IL_002c: Expected O, but got Unknown
			object obj = _arrowSelectorStyeGuiStyleExpand;
			if (obj == null)
			{
				GUIStyle val = new GUIStyle(GUI.skin.label)
				{
					alignment = (TextAnchor)4,
					stretchWidth = true
				};
				_arrowSelectorStyeGuiStyleExpand = val;
				obj = (object)val;
			}
			return (GUIStyle)obj;
		}
	}

	public static GUIStyle ArrowSelectorStyeGuiStyleNoExpand
	{
		get
		{
			//IL_0013: Unknown result type (might be due to invalid IL or missing references)
			//IL_0018: Unknown result type (might be due to invalid IL or missing references)
			//IL_001f: Unknown result type (might be due to invalid IL or missing references)
			//IL_0026: Unknown result type (might be due to invalid IL or missing references)
			//IL_002c: Expected O, but got Unknown
			object obj = _arrowSelectorStyeGuiStyleNoExpand;
			if (obj == null)
			{
				GUIStyle val = new GUIStyle(GUI.skin.label)
				{
					alignment = (TextAnchor)4,
					stretchWidth = false
				};
				_arrowSelectorStyeGuiStyleNoExpand = val;
				obj = (object)val;
			}
			return (GUIStyle)obj;
		}
	}

	public static int HoursPerDay
	{
		get
		{
			if (!GameSettings.KERBIN_TIME)
			{
				return 24;
			}
			return 6;
		}
	}

	public static int DaysPerYear
	{
		get
		{
			if (!GameSettings.KERBIN_TIME)
			{
				return 365;
			}
			return 426;
		}
	}

	public static void SetGUIScale(double s)
	{
		Scale = Mathf.Clamp((float)s, 0.2f, 5f);
		ScaledScreenHeight = Mathf.RoundToInt((float)Screen.height / Scale);
		ScaledScreenWidth = Mathf.RoundToInt((float)Screen.width / Scale);
	}

	public static void CopyDefaultSkin()
	{
		GUI.skin = null;
		DefaultSkin = Object.Instantiate<GUISkin>(GUI.skin);
	}

	public static void CopyCompactSkin()
	{
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Expected O, but got Unknown
		//IL_004a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Expected O, but got Unknown
		//IL_0062: Unknown result type (might be due to invalid IL or missing references)
		//IL_006c: Expected O, but got Unknown
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0084: Expected O, but got Unknown
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_009c: Expected O, but got Unknown
		//IL_00ab: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b5: Expected O, but got Unknown
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cd: Expected O, but got Unknown
		//IL_00db: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e5: Expected O, but got Unknown
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fd: Expected O, but got Unknown
		//IL_010b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0115: Expected O, but got Unknown
		//IL_0123: Unknown result type (might be due to invalid IL or missing references)
		//IL_012d: Expected O, but got Unknown
		//IL_013c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0146: Expected O, but got Unknown
		GUI.skin = null;
		CompactSkin = Object.Instantiate<GUISkin>(GUI.skin);
		((Object)Skin).name = "KSP Compact";
		CompactSkin.label.margin = new RectOffset(1, 1, 1, 1);
		CompactSkin.label.padding = new RectOffset(0, 0, 2, 2);
		CompactSkin.button.margin = new RectOffset(1, 1, 1, 1);
		CompactSkin.button.padding = new RectOffset(4, 4, 2, 2);
		CompactSkin.toggle.margin = new RectOffset(1, 1, 1, 1);
		CompactSkin.toggle.padding = new RectOffset(15, 0, 2, 0);
		CompactSkin.textField.margin = new RectOffset(1, 1, 1, 1);
		CompactSkin.textField.padding = new RectOffset(2, 2, 2, 2);
		CompactSkin.textArea.margin = new RectOffset(1, 1, 1, 1);
		CompactSkin.textArea.padding = new RectOffset(2, 2, 2, 2);
		CompactSkin.window.margin = new RectOffset(0, 0, 0, 0);
		CompactSkin.window.padding = new RectOffset(5, 5, 20, 5);
	}

	private static void CopyTransparentSkin()
	{
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Expected O, but got Unknown
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Expected O, but got Unknown
		GUI.skin = null;
		TransparentSkin = Object.Instantiate<GUISkin>(GUI.skin);
		Texture2D val = new Texture2D(1, 1);
		val.SetPixel(0, 0, new Color(0f, 0f, 0f, 0f));
		val.Apply();
		TransparentSkin.window.normal.background = val;
		TransparentSkin.window.onNormal.background = val;
		TransparentSkin.window.padding = new RectOffset(5, 5, 5, 5);
	}

	public static void LoadSkin(SkinType skinType)
	{
		if ((Object)(object)DefaultSkin == (Object)null)
		{
			CopyDefaultSkin();
		}
		if ((Object)(object)CompactSkin == (Object)null)
		{
			CopyCompactSkin();
		}
		if ((Object)(object)TransparentSkin == (Object)null)
		{
			CopyTransparentSkin();
		}
		switch (skinType)
		{
		case SkinType.DEFAULT:
			Skin = DefaultSkin;
			break;
		case SkinType.MECH_JEB1:
			Skin = AssetBase.GetGUISkin("KSP window 2");
			break;
		case SkinType.COMPACT:
			Skin = CompactSkin;
			break;
		}
	}

	public static GUILayoutOption ExpandWidth(bool b)
	{
		if (!b)
		{
			return LayoutNoExpandWidth;
		}
		return LayoutExpandWidth;
	}

	public static GUILayoutOption LayoutWidth(float width)
	{
		if (_layoutWidthDict.TryGetValue(width, out var value))
		{
			return value;
		}
		value = GUILayout.Width(width);
		_layoutWidthDict.Add(width, value);
		return value;
	}

	public static void SimpleTextField(IEditable ed, float width = 100f, bool expandWidth = false)
	{
		string text = ((!expandWidth) ? GUILayout.TextField(ed.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			LayoutWidth(width),
			ExpandWidth(b: false)
		}) : GUILayout.TextField(ed.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { LayoutWidth(width) }));
		if (text != null && !text.Equals(ed.Text))
		{
			ed.Text = text;
		}
	}

	public static void SimpleTextBox(string? leftLabel, IEditable ed, string? rightLabel = null, float width = 100f, GUIStyle? leftLabelStyle = null, bool horizontalFraming = true, bool expandWidth = false, string? leftLabelTooltip = null)
	{
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		SimpleTextBox(string.IsNullOrEmpty(leftLabel) ? ((GUIContent)null) : ((!string.IsNullOrEmpty(leftLabelTooltip)) ? new GUIContent(leftLabel, leftLabelTooltip) : new GUIContent(leftLabel)), ed, rightLabel, width, leftLabelStyle, horizontalFraming, expandWidth);
	}

	public static void SimpleTextBox(GUIContent? leftLabelContent, IEditable ed, string? rightLabel = null, float width = 100f, GUIStyle? leftLabelStyle = null, bool horizontalFraming = true, bool expandWidth = false)
	{
		if (horizontalFraming)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		}
		if (leftLabelContent != null && !string.IsNullOrEmpty(leftLabelContent.text))
		{
			if (leftLabelStyle == null)
			{
				leftLabelStyle = GUI.skin.label;
			}
			if (expandWidth)
			{
				GUILayout.Label(leftLabelContent, leftLabelStyle, (GUILayoutOption[])(object)new GUILayoutOption[1] { ExpandWidth(b: true) });
			}
			else
			{
				GUILayout.Label(leftLabelContent, leftLabelStyle, Array.Empty<GUILayoutOption>());
			}
		}
		SimpleTextField(ed, width, !expandWidth);
		if (!string.IsNullOrEmpty(rightLabel))
		{
			if (expandWidth)
			{
				GUILayout.Label(rightLabel, (GUILayoutOption[])(object)new GUILayoutOption[1] { ExpandWidth(b: false) });
			}
			else
			{
				GUILayout.Label(rightLabel, Array.Empty<GUILayoutOption>());
			}
		}
		if (horizontalFraming)
		{
			GUILayout.EndHorizontal();
		}
	}

	public static void ToggledTextBox(ref bool toggle, string toggleText, IEditable ed, string? rightLabel = null, GUIStyle? toggleStyle = null, float width = 100f)
	{
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		toggle = ((toggleStyle != null) ? GUILayout.Toggle(toggle, toggleText, toggleStyle, Array.Empty<GUILayoutOption>()) : GUILayout.Toggle(toggle, toggleText, Array.Empty<GUILayoutOption>()));
		SimpleTextField(ed, width);
		if (!string.IsNullOrEmpty(rightLabel))
		{
			GUILayout.Label(rightLabel, Array.Empty<GUILayoutOption>());
		}
		GUILayout.EndHorizontal();
	}

	public static bool ButtonTextBox(string buttonText, IEditable ed, string? rightLabel = null, GUIStyle? buttonStyle = null, float width = 100f)
	{
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		bool result = ((buttonStyle != null) ? GUILayout.Button(buttonText, buttonStyle, Array.Empty<GUILayoutOption>()) : GUILayout.Button(buttonText, Array.Empty<GUILayoutOption>()));
		SimpleTextField(ed, width);
		if (!string.IsNullOrEmpty(rightLabel))
		{
			GUILayout.Label(rightLabel, Array.Empty<GUILayoutOption>());
		}
		GUILayout.EndHorizontal();
		return result;
	}

	public static void SimpleLabel(string leftLabel, string rightLabel = "")
	{
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(leftLabel, (GUILayoutOption[])(object)new GUILayoutOption[1] { ExpandWidth(b: true) });
		if (!string.IsNullOrEmpty(rightLabel))
		{
			GUILayout.Label(rightLabel, (GUILayoutOption[])(object)new GUILayoutOption[1] { ExpandWidth(b: false) });
		}
		GUILayout.EndHorizontal();
	}

	public static void SimpleLabelInt(string leftLabel, int rightValue)
	{
		SimpleLabel(leftLabel, rightValue.ToString());
	}

	public static int ArrowSelector(int index, int numIndices, Action centerGuiAction)
	{
		if (numIndices == 0)
		{
			return index;
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (numIndices > 1 && GUILayout.Button("<", (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.ExpandWidth(false) }))
		{
			index = (index - 1 + numIndices) % numIndices;
		}
		centerGuiAction();
		if (numIndices > 1 && GUILayout.Button(">", (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.ExpandWidth(false) }))
		{
			index = (index + 1) % numIndices;
		}
		GUILayout.EndHorizontal();
		return index;
	}

	public static int ArrowSelector(int index, int modulo, string label, bool expandWidth = true)
	{
		return ArrowSelector(index, modulo, DrawLabel);
		void DrawLabel()
		{
			GUILayout.Label(label, expandWidth ? ArrowSelectorStyeGuiStyleExpand : ArrowSelectorStyeGuiStyleNoExpand, Array.Empty<GUILayoutOption>());
		}
	}

	public static string TimeToDHMS(double seconds, int decimalPlaces = 0)
	{
		if (double.IsInfinity(seconds) || double.IsNaN(seconds))
		{
			return "Inf";
		}
		string text = "";
		bool flag = decimalPlaces > 0;
		try
		{
			string[] array = new string[5] { "y", "d", "h", "m", "s" };
			long[] obj = new long[5] { 0L, 0L, 3600L, 60L, 1L };
			obj[0] = KSPUtil.dateTimeFormatter.Year;
			obj[1] = KSPUtil.dateTimeFormatter.Day;
			long[] array2 = obj;
			if (seconds < 0.0)
			{
				text += "-";
				seconds *= -1.0;
			}
			for (int i = 0; i < array.Length; i++)
			{
				long num = (long)(seconds / (double)array2[i]);
				bool flag2 = text.Length < 2;
				if (!flag2 || num != 0L || (i == array.Length - 1 && text == ""))
				{
					if (!flag2)
					{
						text += " ";
					}
					text = ((flag && seconds < 60.0 && i == array.Length - 1) ? (text + seconds.ToString("00." + new string('0', decimalPlaces))) : ((!flag2) ? (text + num.ToString((i == 1) ? "000" : "00")) : (text + num)));
					text += array[i];
				}
				seconds -= (double)(num * array2[i]);
			}
			return text;
		}
		catch (Exception)
		{
			return "NaN";
		}
	}

	public static bool TryParseDHMS(string s, out double seconds)
	{
		string[] array = new string[5] { "y", "d", "h", "m", "s" };
		int[] obj = new int[5] { 0, 0, 3600, 60, 1 };
		obj[0] = KSPUtil.dateTimeFormatter.Year;
		obj[1] = KSPUtil.dateTimeFormatter.Day;
		int[] array2 = obj;
		s = s.Trim(' ');
		bool flag = s.StartsWith("-");
		seconds = 0.0;
		bool result = false;
		for (int i = 0; i < array.Length; i++)
		{
			s = s.Trim(' ', ',', '-');
			int num = s.IndexOf(array[i]);
			if (num != -1)
			{
				if (!double.TryParse(s.Substring(0, num), out var result2))
				{
					return false;
				}
				seconds += result2 * (double)array2[i];
				s = s.Substring(num + 1);
				result = true;
			}
		}
		if (flag)
		{
			seconds = 0.0 - seconds;
		}
		return result;
	}

	private static double ArcDistance(Vector3 from, Vector3 to)
	{
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		Vector3 position = ((Component)FlightGlobals.ActiveVessel.mainBody).transform.position;
		Vector3 val = position - from;
		double num = ((Vector3)(ref val)).magnitude;
		val = position - to;
		double num2 = ((Vector3)(ref val)).magnitude;
		double num3 = Vector3d.Distance(Vector3d.op_Implicit(from), Vector3d.op_Implicit(to));
		return Math.Acos((num * num + num2 * num2 - num3 * num3) / (2.0 * num * num2)) * FlightGlobals.ActiveVessel.mainBody.Radius;
	}

	public static double FromToETA(Vector3 from, Vector3 to, double speed = 0.0)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		return ArcDistance(from, to) / ((speed > 0.0) ? speed : FlightGlobals.ActiveVessel.horizontalSrfSpeed);
	}

	public static bool MouseIsOverWindow(MechJebCore core)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0032: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0078: Unknown result type (might be due to invalid IL or missing references)
		//IL_007d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		Vector2 val = new Vector2(Input.mousePosition.x, (float)Screen.height - Input.mousePosition.y) / Scale;
		Rect val2;
		if (ComboBox.IsPopupActive)
		{
			val2 = ComboBox.PopupRect;
			if (((Rect)(ref val2)).Contains(val))
			{
				return true;
			}
		}
		foreach (DisplayModule computerModule in core.GetComputerModules<DisplayModule>())
		{
			if (computerModule.Enabled && computerModule.ShowInCurrentScene && !computerModule.IsOverlay)
			{
				val2 = computerModule.WindowPos;
				if (((Rect)(ref val2)).Contains(val))
				{
					return true;
				}
			}
		}
		return false;
	}

	public static Coordinates GetMouseCoordinates(CelestialBody body)
	{
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_008e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0096: Unknown result type (might be due to invalid IL or missing references)
		//IL_009b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ff: Unknown result type (might be due to invalid IL or missing references)
		Ray val = PlanetariumCamera.Camera.ScreenPointToRay(Input.mousePosition);
		((Ray)(ref val)).origin = Vector3d.op_Implicit(ScaledSpace.ScaledToLocalSpace(Vector3d.op_Implicit(((Ray)(ref val)).origin)));
		Vector3d val2 = ((Ray)(ref val)).origin - body.position;
		double num = body.pqsController.radiusMax;
		double num2 = 0.0;
		int num3 = 0;
		Vector3d val3 = default(Vector3d);
		while (num3 < 50)
		{
			if (PQS.LineSphereIntersection(val2, Vector3d.op_Implicit(((Ray)(ref val)).direction), num, ref val3))
			{
				Vector3d val4 = body.position + val3;
				double surfaceHeight = body.pqsController.GetSurfaceHeight(QuaternionD.AngleAxis(body.GetLongitude(val4, false), Vector3d.down) * QuaternionD.AngleAxis(body.GetLatitude(val4, false), Vector3d.forward) * Vector3d.right);
				if (Math.Abs(num - surfaceHeight) < (body.pqsController.radiusMax - body.pqsController.radiusMin) / 100.0)
				{
					return new Coordinates(body.GetLatitude(val4, false), MuUtils.ClampDegrees180(body.GetLongitude(val4, false)));
				}
				num2 = num;
				num = surfaceHeight;
				num3++;
			}
			else
			{
				if (num3 == 0)
				{
					break;
				}
				num = (num2 * 9.0 + num) / 10.0;
				num3++;
			}
		}
		return null;
	}

	public static void RecordTooltip(int windowId)
	{
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Invalid comparison between Unknown and I4
		if ((int)Event.current.type == 7)
		{
			_tooltipTexts.TryGetValue(windowId, out var value);
			if (value == null)
			{
				value = string.Empty;
			}
			if (!(GUI.tooltip == value))
			{
				_tooltipChanged = true;
				_tooltipBeginDt = DateTime.UtcNow;
				_tooltipTexts[windowId] = GUI.tooltip;
			}
		}
	}

	public static void ShowTooltip(int windowId)
	{
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Expected O, but got Unknown
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		//IL_008f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0099: Expected O, but got Unknown
		//IL_0099: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ac: Expected O, but got Unknown
		//IL_016c: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cd: Expected O, but got Unknown
		//IL_00fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0114: Unknown result type (might be due to invalid IL or missing references)
		//IL_0157: Unknown result type (might be due to invalid IL or missing references)
		//IL_015c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0196: Unknown result type (might be due to invalid IL or missing references)
		//IL_0185: Unknown result type (might be due to invalid IL or missing references)
		//IL_018a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0190: Expected O, but got Unknown
		if (_tooltipTexts.TryGetValue(windowId, out var value) && !string.IsNullOrEmpty(value) && !((DateTime.UtcNow - _tooltipBeginDt).TotalMilliseconds <= 1000.0))
		{
			if (_tooltipStyle == null)
			{
				Texture2D val = new Texture2D(1, 1, (TextureFormat)5, false)
				{
					hideFlags = (HideFlags)61
				};
				val.SetPixel(0, 0, new Color(0.1f, 0.1f, 0.1f, 1f));
				val.Apply();
				_tooltipStyle = new GUIStyle(GUI.skin.box)
				{
					padding = new RectOffset(3, 3, 3, 3),
					alignment = (TextAnchor)4,
					wordWrap = true
				};
				_tooltipStyle.normal.background = val;
			}
			if (_tooltipChanged)
			{
				GUIContent val2 = new GUIContent(value);
				float num = default(float);
				float val3 = default(float);
				_tooltipStyle.CalcMinMaxWidth(val2, ref num, ref val3);
				val3 = Math.Min(val3, 200f);
				float num2 = _tooltipStyle.CalcHeight(val2, 200f);
				float num3 = Input.mousePosition.x / Scale;
				float num4 = ((float)Screen.height - Input.mousePosition.y) / Scale;
				_tooltipRect = new Rect(Math.Min((float)ScaledScreenWidth - val3, num3 + 15f), Math.Min((float)ScaledScreenHeight - num2, num4 + 10f), val3, num2);
				_tooltipChanged = false;
			}
			int tooltipWindowId = _tooltipWindowId;
			Rect tooltipRect = _tooltipRect;
			object obj = _003C_003Ec._003C_003E9__94_0;
			if (obj == null)
			{
				WindowFunction val4 = delegate
				{
				};
				_003C_003Ec._003C_003E9__94_0 = val4;
				obj = (object)val4;
			}
			GUI.Window(tooltipWindowId, tooltipRect, (WindowFunction)obj, value, _tooltipStyle);
			GUI.BringWindowToFront(_tooltipWindowId);
		}
	}
}
