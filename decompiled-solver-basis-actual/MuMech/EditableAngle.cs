using System;
using UnityEngine;

namespace MuMech;

public class EditableAngle
{
	public enum Direction
	{
		NS,
		EW
	}

	[Persistent]
	public readonly EditableDouble Degrees;

	[Persistent]
	public readonly EditableDouble Minutes;

	[Persistent]
	public readonly EditableDouble Seconds;

	[Persistent]
	public bool Negative;

	public EditableAngle(double angle)
	{
		angle = MuUtils.ClampDegrees180(angle);
		Negative = angle < 0.0;
		angle = Math.Abs(angle);
		Degrees = new EditableDouble((int)angle);
		angle -= Degrees.Val;
		Minutes = new EditableDouble((int)(60.0 * angle));
		angle -= Minutes.Val / 60.0;
		Seconds = new EditableDouble(Math.Round(3600.0 * angle));
	}

	public static implicit operator double(EditableAngle x)
	{
		return (double)((!x.Negative) ? 1 : (-1)) * ((double)x.Degrees + (double)x.Minutes / 60.0 + (double)x.Seconds / 3600.0);
	}

	public static implicit operator EditableAngle(double x)
	{
		return new EditableAngle(x);
	}

	public void DrawEditGUI(Direction direction)
	{
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		Degrees.Text = GUILayout.TextField(Degrees.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.Width(30f) });
		GUILayout.Label("°", (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.ExpandWidth(false) });
		Minutes.Text = GUILayout.TextField(Minutes.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.Width(30f) });
		GUILayout.Label("'", (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.ExpandWidth(false) });
		Seconds.Text = GUILayout.TextField(Seconds.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.Width(30f) });
		GUILayout.Label("\"", (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.ExpandWidth(false) });
		if (GUILayout.Button((direction != 0) ? (Negative ? "W" : "E") : (Negative ? "S" : "N"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.Width(25f) }))
		{
			Negative = !Negative;
		}
		GUILayout.EndHorizontal();
	}
}
