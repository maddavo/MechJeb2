using System;
using UnityEngine;

namespace MuMech;

public class PlotArea
{
	public delegate void AreaChanged(double minx, double maxx, double miny, double maxy);

	private static readonly GUIStyle _selectionStyle;

	public bool Draggable = true;

	public int[] HoveredPoint;

	public int[] SelectedPoint;

	private bool _mouseDown;

	private readonly double _minx;

	private readonly double _maxx;

	private readonly double _miny;

	private readonly double _maxy;

	private readonly Texture2D _texture;

	private readonly AreaChanged _callback;

	private int[] _lastHoveredPoint;

	public PlotArea(double minx, double maxx, double miny, double maxy, Texture2D texture, AreaChanged callback)
	{
		_minx = minx;
		_maxx = maxx;
		_miny = miny;
		_maxy = maxy;
		_texture = texture;
		_callback = callback;
	}

	static PlotArea()
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Expected O, but got Unknown
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Expected O, but got Unknown
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		_selectionStyle = new GUIStyle();
		Texture2D val = new Texture2D(1, 1, (TextureFormat)4, false);
		val.SetPixel(0, 0, new Color(0f, 0f, 1f, 0.3f));
		val.Apply();
		_selectionStyle.normal.background = val;
	}

	private double X(int index)
	{
		return _minx + (double)index * (_maxx - _minx) / (double)((Texture)_texture).width;
	}

	private double Y(int index)
	{
		return _miny + (double)index * (_maxy - _miny) / (double)((Texture)_texture).height;
	}

	private void ZoomSelectionBox(int[] hoveredPoint)
	{
		_mouseDown = false;
		if (Math.Abs(SelectedPoint[0] - hoveredPoint[0]) > 5 && Math.Abs(SelectedPoint[1] - hoveredPoint[1]) > 5)
		{
			Debug.Log((object)"[MechJeb] porkchop plotter, zooming plotarea");
			_callback(X(Math.Min(SelectedPoint[0], hoveredPoint[0])), X(Math.Max(SelectedPoint[0], hoveredPoint[0])), Y(Math.Min(SelectedPoint[1], hoveredPoint[1])), Y(Math.Max(SelectedPoint[1], hoveredPoint[1])));
		}
	}

	public void DoGUI()
	{
		//IL_0043: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Invalid comparison between Unknown and I4
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_008e: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ca: Expected I4, but got Unknown
		//IL_0096: Unknown result type (might be due to invalid IL or missing references)
		//IL_0099: Unknown result type (might be due to invalid IL or missing references)
		//IL_009e: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_00be: Unknown result type (might be due to invalid IL or missing references)
		//IL_0226: Unknown result type (might be due to invalid IL or missing references)
		//IL_023f: Unknown result type (might be due to invalid IL or missing references)
		//IL_016d: Unknown result type (might be due to invalid IL or missing references)
		GUILayout.Box((Texture)(object)_texture, GUIStyle.none, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(((Texture)_texture).width),
			GUILayout.Height((float)((Texture)_texture).height)
		});
		if ((int)Event.current.type == 7)
		{
			HoveredPoint = null;
			Rect lastRect = GUILayoutUtility.GetLastRect();
			((Rect)(ref lastRect)).x = ((Rect)(ref lastRect)).x + 1f;
			((Rect)(ref lastRect)).y = ((Rect)(ref lastRect)).y + 2f;
			Vector2 mousePosition = Event.current.mousePosition;
			if (((Rect)(ref lastRect)).Contains(mousePosition))
			{
				Vector2 val = mousePosition - ((Rect)(ref lastRect)).position;
				HoveredPoint = new int[2]
				{
					(int)val.x,
					(int)(((Rect)(ref lastRect)).height - val.y - 1f)
				};
			}
			else if (_mouseDown && _lastHoveredPoint != null)
			{
				ZoomSelectionBox(_lastHoveredPoint);
			}
			if (_mouseDown)
			{
				GUI.Box(new Rect(((Rect)(ref lastRect)).x + (float)Math.Min(SelectedPoint[0], HoveredPoint[0]), ((Rect)(ref lastRect)).y + ((Rect)(ref lastRect)).height - (float)Math.Max(SelectedPoint[1], HoveredPoint[1]), (float)Math.Abs(SelectedPoint[0] - HoveredPoint[0]), (float)Math.Abs(SelectedPoint[1] - HoveredPoint[1])), "", _selectionStyle);
			}
		}
		Draggable = HoveredPoint == null;
		if (HoveredPoint != null)
		{
			EventType type = Event.current.type;
			switch ((int)type)
			{
			case 0:
				if (Event.current.button == 0)
				{
					_mouseDown = true;
					SelectedPoint = HoveredPoint;
				}
				break;
			case 1:
				if (Event.current.button == 0 && _mouseDown)
				{
					ZoomSelectionBox(HoveredPoint);
				}
				break;
			case 6:
				if (Event.current.delta.y != 0f)
				{
					double num = ((Event.current.delta.y < 0f) ? 0.7 : 1.4285714285714286);
					double num2 = _maxx - _minx;
					double num3 = _maxy - _miny;
					double num4 = X(HoveredPoint[0]) - (double)HoveredPoint[0] * num * num2 / (double)((Texture)_texture).width;
					double num5 = Y(HoveredPoint[1]) - (double)HoveredPoint[1] * num * num3 / (double)((Texture)_texture).height;
					_callback(num4, num4 + num * num2, num5, num5 + num * num3);
				}
				break;
			case 5:
				ProcessKeys();
				break;
			}
		}
		_lastHoveredPoint = HoveredPoint;
	}

	private void ProcessKeys()
	{
		//IL_0005: Unknown result type (might be due to invalid IL or missing references)
		//IL_000a: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Invalid comparison between Unknown and I4
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Invalid comparison between Unknown and I4
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Invalid comparison between Unknown and I4
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Invalid comparison between Unknown and I4
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Invalid comparison between Unknown and I4
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Invalid comparison between Unknown and I4
		//IL_003d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Invalid comparison between Unknown and I4
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_005a: Invalid comparison between Unknown and I4
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Invalid comparison between Unknown and I4
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0062: Invalid comparison between Unknown and I4
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Invalid comparison between Unknown and I4
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Invalid comparison between Unknown and I4
		KeyCode keyCode = Event.current.keyCode;
		int num = 0;
		int num2 = 0;
		if ((int)keyCode == 119 || (int)keyCode == 115 || (int)keyCode == 273 || (int)keyCode == 274)
		{
			num2 = (((int)keyCode == 119 || (int)keyCode == 273) ? 1 : (-1));
		}
		else if ((int)keyCode == 97 || (int)keyCode == 100 || (int)keyCode == 276 || (int)keyCode == 275)
		{
			num = (((int)keyCode == 100 || (int)keyCode == 275) ? 1 : (-1));
		}
		if (num != 0 || num2 != 0)
		{
			double num3 = _maxx - _minx;
			double num4 = _maxy - _miny;
			double num5 = num3 * 0.25 * (double)num;
			double num6 = num4 * 0.25 * (double)num2;
			double num7 = _minx + num5;
			double num8 = _miny + num6;
			_callback(num7, num7 + num3, num8, num8 + num4);
		}
	}
}
