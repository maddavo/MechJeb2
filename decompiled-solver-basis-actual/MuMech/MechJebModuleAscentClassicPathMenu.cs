using System;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleAscentClassicPathMenu : DisplayModule
{
	private MechJebModuleAscentSettings _ascentSettings;

	private MechJebModuleAscentClassicAutopilot _path;

	private static readonly Texture2D _pathTexture = new Texture2D(400, 100);

	private MechJebModuleFlightRecorder _recorder;

	private double _lastMaxAtmosphereAltitude = -1.0;

	public MechJebModuleAscentClassicPathMenu(MechJebCore core)
		: base(core)
	{
		Hidden = true;
	}

	public override void OnStart(StartState state)
	{
		_recorder = Core.GetComputerModule<MechJebModuleFlightRecorder>();
		_ascentSettings = Core.GetComputerModule<MechJebModuleAscentSettings>();
		_path = Core.GetComputerModule<MechJebModuleAscentClassicAutopilot>();
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(300f),
			GUILayout.Height(100f)
		};
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_03e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_03eb: Invalid comparison between Unknown and I4
		//IL_03f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_03f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_04b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_04c1: Unknown result type (might be due to invalid IL or missing references)
		//IL_04cc: Unknown result type (might be due to invalid IL or missing references)
		if (_lastMaxAtmosphereAltitude != base.MainBody.RealMaxAtmosphereAltitude())
		{
			_lastMaxAtmosphereAltitude = base.MainBody.RealMaxAtmosphereAltitude();
			UpdateAtmoTexture(_pathTexture, base.MainBody, _ascentSettings.AutoPath ? _ascentSettings.AutoTurnEndAltitude : ((double)_ascentSettings.TurnEndAltitude));
		}
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		double num = _ascentSettings.TurnShapeExponent;
		_ascentSettings.AutoPath = GUILayout.Toggle(_ascentSettings.AutoPath, Localizer.Format("#MechJeb_AscentPathEd_auto"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		if (_ascentSettings.AutoPath)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AscentPathEd_label1"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(60f) });
			_ascentSettings.AutoTurnPerc = Mathf.Floor(GUILayout.HorizontalSlider(_ascentSettings.AutoTurnPerc * 200f, 1f, 210.5f, Array.Empty<GUILayoutOption>())) / 200f;
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AscentPathEd_label2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(60f) });
			_ascentSettings.AutoTurnSpdFactor = Mathf.Floor(GUILayout.HorizontalSlider(_ascentSettings.AutoTurnSpdFactor * 2f, 8f, 160f, Array.Empty<GUILayoutOption>())) / 2f;
			GUILayout.EndHorizontal();
		}
		if (_ascentSettings.AutoPath)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AscentPathEd_label3"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.Label(Statics.ToSI(_ascentSettings.AutoTurnStartAltitude, 2, int.MaxValue) + "m ", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.Label(Localizer.Format("#MechJeb_AscentPathEd_label4"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.Label(Statics.ToSI(_ascentSettings.AutoTurnStartVelocity, 3, int.MaxValue) + "m/s", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AscentPathEd_label5"), Array.Empty<GUILayoutOption>());
			GUILayout.Label(Statics.ToSI(_ascentSettings.AutoTurnEndAltitude, 2, int.MaxValue) + "m", GuiUtils.MiddleRightLabel, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.EndHorizontal();
		}
		else
		{
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_AscentPathEd_label6"), _ascentSettings.TurnStartAltitude, "km");
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_AscentPathEd_label7"), _ascentSettings.TurnStartVelocity, "m/s");
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_AscentPathEd_label8"), _ascentSettings.TurnEndAltitude, "km");
		}
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_AscentPathEd_label9"), _ascentSettings.TurnEndAngle, "°");
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_AscentPathEd_label10"), _ascentSettings.TurnShapeExponent, "%");
		double num2 = GUILayout.HorizontalSlider((float)(double)_ascentSettings.TurnShapeExponent, 0f, 1f, Array.Empty<GUILayoutOption>());
		if (Math.Round(Math.Abs(num2 - num), 3) > 0.0)
		{
			_ascentSettings.TurnShapeExponent.Val = Math.Round(num2, 3);
		}
		GUILayout.Box((Texture)(object)_pathTexture, Array.Empty<GUILayoutOption>());
		if ((int)Event.current.type == 7)
		{
			Rect lastRect = GUILayoutUtility.GetLastRect();
			((Rect)(ref lastRect)).xMin = ((Rect)(ref lastRect)).xMin + (float)GUI.skin.box.margin.left;
			((Rect)(ref lastRect)).yMin = ((Rect)(ref lastRect)).yMin + (float)GUI.skin.box.margin.top;
			((Rect)(ref lastRect)).xMax = ((Rect)(ref lastRect)).xMax - (float)GUI.skin.box.margin.right;
			((Rect)(ref lastRect)).yMax = ((Rect)(ref lastRect)).yMax - (float)GUI.skin.box.margin.bottom;
			float num3 = (float)((_ascentSettings.AutoPath ? _ascentSettings.AutoTurnEndAltitude : ((double)_ascentSettings.TurnEndAltitude)) / (double)((Rect)(ref lastRect)).height);
			DrawnPath(lastRect, num3, num3, _path, Color.red);
			DrawnTrajectory(lastRect, _recorder);
		}
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	public static void UpdateAtmoTexture(Texture2D texture, CelestialBody mainBody, double maxAltitude, bool realAtmo = false)
	{
		//IL_007a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		double num = maxAltitude / (double)((Texture)texture).height;
		double num2 = mainBody.RealMaxAtmosphereAltitude();
		double atmospherePressureSeaLevel = mainBody.atmospherePressureSeaLevel;
		Color val = default(Color);
		for (int i = 0; i < ((Texture)texture).height; i++)
		{
			double num3 = num * (double)i;
			num3 = ((!realAtmo) ? (1.0 - num3 / num2) : (mainBody.GetPressure(num3) / atmospherePressureSeaLevel));
			float num4 = (float)(mainBody.atmosphere ? num3 : 0.0);
			((Color)(ref val))._002Ector(0f, 0f, num4);
			for (int j = 0; j < ((Texture)texture).width; j++)
			{
				texture.SetPixel(j, i, val);
				if (mainBody.atmosphere && (int)(num2 / num) == i)
				{
					texture.SetPixel(j, i, XKCDColors.LightGreyBlue);
				}
			}
		}
		texture.Apply();
	}

	private void DrawnPath(Rect r, float scaleX, float scaleY, MechJebModuleAscentClassicAutopilot path, Color color)
	{
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00be: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_00db: Unknown result type (might be due to invalid IL or missing references)
		float num = 0f;
		float num2 = 0f;
		Vector2 val = default(Vector2);
		((Vector2)(ref val))._002Ector(((Rect)(ref r)).xMin, ((Rect)(ref r)).yMax);
		Vector2 val2 = default(Vector2);
		while ((double)num < (_ascentSettings.AutoPath ? _ascentSettings.AutoTurnEndAltitude : ((double)_ascentSettings.TurnEndAltitude)) && num2 < ((Rect)(ref r)).width * scaleX)
		{
			float num3 = (float)(((double)num < path.VerticalAscentEnd()) ? 90.0 : path.FlightPathAngle(num, 0.0));
			num += scaleY * Mathf.Sin(num3 * ((float)Math.PI / 180f));
			num2 += scaleX * Mathf.Cos(num3 * ((float)Math.PI / 180f));
			val2.x = ((Rect)(ref r)).xMin + num2 / scaleX;
			val2.y = ((Rect)(ref r)).yMax - num / scaleY;
			Vector2 val3 = val - val2;
			if ((double)((Vector2)(ref val3)).sqrMagnitude >= 1.0)
			{
				Drawing.DrawLine(val, val2, color, 2f, true);
				val.x = val2.x;
				val.y = val2.y;
			}
		}
	}

	private void DrawnTrajectory(Rect r, MechJebModuleFlightRecorder recorder)
	{
		//IL_008e: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_0105: Unknown result type (might be due to invalid IL or missing references)
		//IL_0106: Unknown result type (might be due to invalid IL or missing references)
		//IL_0107: Unknown result type (might be due to invalid IL or missing references)
		//IL_0119: Unknown result type (might be due to invalid IL or missing references)
		//IL_0126: Unknown result type (might be due to invalid IL or missing references)
		if (recorder.History.Length <= 2 || recorder.HistoryIdx == 0)
		{
			return;
		}
		float num = (float)((_ascentSettings.AutoPath ? _ascentSettings.AutoTurnEndAltitude : ((double)_ascentSettings.TurnEndAltitude)) / (double)((Rect)(ref r)).height);
		int i = 1;
		Vector2 val = default(Vector2);
		((Vector2)(ref val))._002Ector(((Rect)(ref r)).xMin + (float)(recorder.History[0].DownRange / (double)num), ((Rect)(ref r)).yMax - (float)(recorder.History[0].AltitudeASL / (double)num));
		Vector2 val2 = default(Vector2);
		for (; i <= recorder.HistoryIdx && i < recorder.History.Length; i++)
		{
			MechJebModuleFlightRecorder.RecordStruct recordStruct = recorder.History[i];
			val2.x = ((Rect)(ref r)).xMin + (float)(recordStruct.DownRange / (double)num);
			val2.y = ((Rect)(ref r)).yMax - (float)(recordStruct.AltitudeASL / (double)num);
			if (((Rect)(ref r)).Contains(val2))
			{
				Vector2 val3 = val - val2;
				if ((double)((Vector2)(ref val3)).sqrMagnitude >= 1.0)
				{
					goto IL_0105;
				}
			}
			if (i >= 2)
			{
				continue;
			}
			goto IL_0105;
			IL_0105:
			Drawing.DrawLine(val, val2, Color.white, 2f, true);
			val.x = val2.x;
			val.y = val2.y;
		}
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_AscentPathEd_title");
	}

	public override string IconName()
	{
		return "Ascent Path Editor";
	}
}
