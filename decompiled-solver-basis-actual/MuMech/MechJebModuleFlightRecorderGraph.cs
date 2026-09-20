using System;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleFlightRecorderGraph : DisplayModule
{
	public struct graphState
	{
		public double minimum;

		public double maximum;

		public string[] labels;

		public double[] labelsPos;

		public int labelsActive;

		public bool display;

		public void Reset()
		{
			minimum = 0.0;
			maximum = 0.0;
			labels = new string[11];
			labelsPos = new double[11];
		}
	}

	private const int ScaleTicks = 11;

	[Persistent(pass = 4)]
	public bool downrange = true;

	[Persistent(pass = 4)]
	public bool realAtmo;

	[Persistent(pass = 4)]
	public bool stages;

	[Persistent(pass = 4)]
	public int hSize = 4;

	[Persistent(pass = 4)]
	public int vSize = 2;

	[Persistent(pass = 4)]
	public bool autoScale = true;

	[Persistent(pass = 4)]
	public int timeScale;

	[Persistent(pass = 4)]
	public int downrangeScale;

	[Persistent(pass = 4)]
	public int scaleIdx;

	public bool ascentPath;

	private static Texture2D backgroundTexture;

	private CelestialBody oldMainBody;

	private static readonly int typeCount = Enum.GetValues(typeof(MechJebModuleFlightRecorder.RecordType)).Length;

	private readonly graphState[] graphStates;

	private double lastMaximumAltitude;

	private readonly double precision = 0.2;

	private int width = 512;

	private int height = 256;

	private bool paused;

	private float hPos;

	private bool follow = true;

	private MechJebModuleFlightRecorder recorder;

	public MechJebModuleFlightRecorderGraph(MechJebCore core)
		: base(core)
	{
		Priority = 2000;
		graphStates = new graphState[typeCount];
	}

	public override void OnStart(StartState state)
	{
		if (!HighLogic.LoadedSceneIsEditor)
		{
			width = 128 * hSize;
			height = 128 * vSize;
			recorder = Core.GetComputerModule<MechJebModuleFlightRecorder>();
			ResetScale();
		}
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_008e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Expected O, but got Unknown
		//IL_0642: Unknown result type (might be due to invalid IL or missing references)
		//IL_0647: Unknown result type (might be due to invalid IL or missing references)
		//IL_0672: Unknown result type (might be due to invalid IL or missing references)
		//IL_06bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0706: Unknown result type (might be due to invalid IL or missing references)
		//IL_074f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0798: Unknown result type (might be due to invalid IL or missing references)
		//IL_07e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_0839: Unknown result type (might be due to invalid IL or missing references)
		//IL_0882: Unknown result type (might be due to invalid IL or missing references)
		//IL_08cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0918: Unknown result type (might be due to invalid IL or missing references)
		//IL_0963: Unknown result type (might be due to invalid IL or missing references)
		//IL_09ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_09f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a3f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0a8a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ad5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b2d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b32: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b35: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b46: Unknown result type (might be due to invalid IL or missing references)
		//IL_0b4b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bb4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c01: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c4f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0c9b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ce7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0d33: Unknown result type (might be due to invalid IL or missing references)
		//IL_0d7f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0dcc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0e1b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0e6a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0eb9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0f08: Unknown result type (might be due to invalid IL or missing references)
		//IL_0f52: Unknown result type (might be due to invalid IL or missing references)
		//IL_0fa1: Unknown result type (might be due to invalid IL or missing references)
		//IL_102b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0ff0: Unknown result type (might be due to invalid IL or missing references)
		//IL_10c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_10cd: Invalid comparison between Unknown and I4
		//IL_10ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_1115: Unknown result type (might be due to invalid IL or missing references)
		//IL_1138: Unknown result type (might be due to invalid IL or missing references)
		//IL_1149: Unknown result type (might be due to invalid IL or missing references)
		//IL_1168: Unknown result type (might be due to invalid IL or missing references)
		//IL_117a: Unknown result type (might be due to invalid IL or missing references)
		//IL_1198: Unknown result type (might be due to invalid IL or missing references)
		//IL_11a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_11c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_11d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_11f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_1207: Unknown result type (might be due to invalid IL or missing references)
		//IL_1225: Unknown result type (might be due to invalid IL or missing references)
		//IL_1236: Unknown result type (might be due to invalid IL or missing references)
		//IL_1254: Unknown result type (might be due to invalid IL or missing references)
		//IL_1265: Unknown result type (might be due to invalid IL or missing references)
		//IL_1284: Unknown result type (might be due to invalid IL or missing references)
		//IL_1296: Unknown result type (might be due to invalid IL or missing references)
		//IL_12b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_12c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_12e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_12f8: Unknown result type (might be due to invalid IL or missing references)
		//IL_1317: Unknown result type (might be due to invalid IL or missing references)
		//IL_1329: Unknown result type (might be due to invalid IL or missing references)
		//IL_1348: Unknown result type (might be due to invalid IL or missing references)
		//IL_135a: Unknown result type (might be due to invalid IL or missing references)
		//IL_1379: Unknown result type (might be due to invalid IL or missing references)
		//IL_138b: Unknown result type (might be due to invalid IL or missing references)
		//IL_13aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_13bc: Unknown result type (might be due to invalid IL or missing references)
		//IL_13db: Unknown result type (might be due to invalid IL or missing references)
		//IL_13ed: Unknown result type (might be due to invalid IL or missing references)
		if ((Object)(object)oldMainBody != (Object)(object)base.MainBody || lastMaximumAltitude != graphStates[2].maximum || height != ((Texture)backgroundTexture).height)
		{
			UpdateScale();
			lastMaximumAltitude = graphStates[2].maximum;
			if ((Object)(object)backgroundTexture == (Object)null || height != ((Texture)backgroundTexture).height)
			{
				Object.Destroy((Object)(object)backgroundTexture);
				backgroundTexture = new Texture2D(1, height);
			}
			MechJebModuleAscentClassicPathMenu.UpdateAtmoTexture(backgroundTexture, base.Vessel.mainBody, lastMaximumAltitude, realAtmo);
			oldMainBody = base.MainBody;
		}
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(paused ? Localizer.Format("#MechJeb_Flightrecord_Button1_1") : Localizer.Format("#MechJeb_Flightrecord_Button1_2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			paused = !paused;
		}
		if (GUILayout.Button(downrange ? Localizer.Format("#MechJeb_Flightrecord_Button2_1") : Localizer.Format("#MechJeb_Flightrecord_Button2_2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			downrange = !downrange;
		}
		GUILayout.Label(Localizer.Format("#MechJeb_Flightrecord_Label1", new string[1] { GuiUtils.TimeToDHMS(recorder.TimeSinceMark) }), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.Label(Localizer.Format("#MechJeb_Flightrecord_Label2", new string[1] { Statics.ToSI(recorder.History[recorder.HistoryIdx].DownRange, 4, int.MaxValue) }) + "m", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.FlexibleSpace();
		if (GUILayout.Button(Localizer.Format("#MechJeb_Flightrecord_Button3"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			ResetScale();
			recorder.Mark();
		}
		if (GUILayout.Button(Localizer.Format("#MechJeb_Flightrecord_Button4"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			ResetScale();
		}
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		autoScale = GUILayout.Toggle(autoScale, Localizer.Format("#MechJeb_Flightrecord_checkbox1"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		if (!autoScale && GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			if (downrange)
			{
				downrangeScale--;
			}
			else
			{
				timeScale--;
			}
		}
		float num = (float)(downrange ? recorder.Maximums[3] : recorder.Maximums[0]);
		double num2 = Math.Max(Math.Ceiling(Math.Log((downrange ? ((double)num) : ((double)num / precision)) / (double)width, 2.0)), 0.0);
		double num3 = (downrange ? downrangeScale : timeScale);
		double y = (autoScale ? num2 : num3);
		double num4 = (downrange ? Math.Pow(2.0, y) : (precision * Math.Pow(2.0, y)));
		GUILayout.Label(downrange ? (Statics.ToSI(num4, 2, int.MaxValue) + "m/px") : (GuiUtils.TimeToDHMS(num4, 1) + "/px"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		if (!autoScale && GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			if (downrange)
			{
				downrangeScale++;
			}
			else
			{
				timeScale++;
			}
		}
		if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			hSize--;
		}
		GUILayout.Label(width.ToString(), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			hSize++;
		}
		GUILayout.Label("x", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			vSize--;
		}
		GUILayout.Label(height.ToString(), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			vSize++;
		}
		timeScale = Mathf.Clamp(timeScale, 0, 20);
		downrangeScale = Mathf.Clamp(downrangeScale, 0, 20);
		hSize = Mathf.Clamp(hSize, 1, 20);
		vSize = Mathf.Clamp(vSize, 1, 10);
		bool flag = realAtmo;
		realAtmo = GUILayout.Toggle(realAtmo, Localizer.Format("#MechJeb_Flightrecord_checkbox2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		if (flag != realAtmo)
		{
			MechJebModuleAscentClassicPathMenu.UpdateAtmoTexture(backgroundTexture, base.Vessel.mainBody, lastMaximumAltitude, realAtmo);
		}
		GUILayout.FlexibleSpace();
		if (GUILayout.Button(Localizer.Format("#MechJeb_Flightrecord_Button5"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			recorder.DumpCsv();
		}
		GUILayout.Label(Localizer.Format("#MechJeb_Flightrecord_Label3", new string[1] { ((float)(100 * recorder.HistoryIdx) / (float)recorder.History.Length).ToString("F1") }), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		Color color = GUI.color;
		stages = GUILayout.Toggle(stages, Localizer.Format("#MechJeb_Flightrecord_checkbox3"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.White;
		graphStates[2].display = GUILayout.Toggle(graphStates[2].display, Localizer.Format("#MechJeb_Flightrecord_checkbox4"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Grey;
		graphStates[12].display = GUILayout.Toggle(graphStates[12].display, Localizer.Format("#MechJeb_Flightrecord_checkbox5"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.LightRed;
		graphStates[7].display = GUILayout.Toggle(graphStates[7].display, Localizer.Format("#MechJeb_Flightrecord_checkbox6"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Yellow;
		graphStates[4].display = GUILayout.Toggle(graphStates[4].display, Localizer.Format("#MechJeb_Flightrecord_checkbox7"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Apricot;
		graphStates[5].display = GUILayout.Toggle(graphStates[5].display, Localizer.Format("#MechJeb_Flightrecord_checkbox8"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Pink;
		graphStates[6].display = GUILayout.Toggle(graphStates[6].display, Localizer.Format("#MechJeb_Flightrecord_checkbox9"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUI.color = XKCDColors.Cyan;
		graphStates[8].display = GUILayout.Toggle(graphStates[8].display, Localizer.Format("#MechJeb_Flightrecord_checkbox10"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Lavender;
		graphStates[9].display = GUILayout.Toggle(graphStates[9].display, Localizer.Format("#MechJeb_Flightrecord_checkbox11"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Lime;
		graphStates[10].display = GUILayout.Toggle(graphStates[10].display, Localizer.Format("#MechJeb_Flightrecord_checkbox12"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Orange;
		graphStates[11].display = GUILayout.Toggle(graphStates[11].display, Localizer.Format("#MechJeb_Flightrecord_checkbox13"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Mint;
		graphStates[13].display = GUILayout.Toggle(graphStates[13].display, Localizer.Format("#MechJeb_Flightrecord_checkbox14"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Beige;
		graphStates[17].display = GUILayout.Toggle(graphStates[17].display, "ΔV", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Green;
		graphStates[14].display = GUILayout.Toggle(graphStates[14].display, Localizer.Format("#MechJeb_Flightrecord_checkbox15"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.LightBrown;
		graphStates[15].display = GUILayout.Toggle(graphStates[15].display, Localizer.Format("#MechJeb_Flightrecord_checkbox16"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = XKCDColors.Cerise;
		graphStates[16].display = GUILayout.Toggle(graphStates[16].display, Localizer.Format("#MechJeb_Flightrecord_checkbox17"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUI.color = color;
		GUILayout.EndHorizontal();
		GUILayout.Space(10f);
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Space(50f);
		GUILayout.Box((Texture)(object)Texture2D.blackTexture, (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(width),
			GUILayout.Height((float)height)
		});
		Rect lastRect = GUILayoutUtility.GetLastRect();
		DrawScaleLabels(lastRect);
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		color = GUI.color;
		if (!graphStates[scaleIdx].display)
		{
			int i;
			for (i = 0; i < typeCount && !graphStates[i].display; i++)
			{
			}
			if (i == typeCount)
			{
				i = 0;
			}
			scaleIdx = i;
		}
		if (graphStates[2].display)
		{
			GUI.color = XKCDColors.White;
			if (GUILayout.Toggle(scaleIdx == 2, Localizer.Format("#MechJeb_Flightrecord_checkbox18"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 2;
			}
		}
		if (graphStates[12].display)
		{
			GUI.color = XKCDColors.Grey;
			if (GUILayout.Toggle(scaleIdx == 12, Localizer.Format("#MechJeb_Flightrecord_checkbox19"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 12;
			}
		}
		if (graphStates[7].display)
		{
			GUI.color = XKCDColors.LightRed;
			if (GUILayout.Toggle(scaleIdx == 7, Localizer.Format("#MechJeb_Flightrecord_checkbox20"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 7;
			}
		}
		if (graphStates[4].display)
		{
			GUI.color = XKCDColors.Yellow;
			if (GUILayout.Toggle(scaleIdx == 4, Localizer.Format("#MechJeb_Flightrecord_checkbox21"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 4;
			}
		}
		if (graphStates[5].display)
		{
			GUI.color = XKCDColors.Apricot;
			if (GUILayout.Toggle(scaleIdx == 5, Localizer.Format("#MechJeb_Flightrecord_checkbox22"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 5;
			}
		}
		if (graphStates[6].display)
		{
			GUI.color = XKCDColors.Pink;
			if (GUILayout.Toggle(scaleIdx == 6, Localizer.Format("#MechJeb_Flightrecord_checkbox23"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 6;
			}
		}
		if (graphStates[8].display)
		{
			GUI.color = XKCDColors.Cyan;
			if (GUILayout.Toggle(scaleIdx == 8, Localizer.Format("#MechJeb_Flightrecord_checkbox24"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 8;
			}
		}
		if (graphStates[9].display)
		{
			GUI.color = XKCDColors.Lavender;
			if (GUILayout.Toggle(scaleIdx == 9, Localizer.Format("#MechJeb_Flightrecord_checkbox25"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 9;
			}
		}
		if (graphStates[10].display)
		{
			GUI.color = XKCDColors.Lime;
			if (GUILayout.Toggle(scaleIdx == 10, Localizer.Format("#MechJeb_Flightrecord_checkbox26"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 10;
			}
		}
		if (graphStates[11].display)
		{
			GUI.color = XKCDColors.Orange;
			if (GUILayout.Toggle(scaleIdx == 11, Localizer.Format("#MechJeb_Flightrecord_checkbox27"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 11;
			}
		}
		if (graphStates[13].display)
		{
			GUI.color = XKCDColors.Mint;
			if (GUILayout.Toggle(scaleIdx == 13, Localizer.Format("#MechJeb_Flightrecord_checkbox28"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 13;
			}
		}
		if (graphStates[17].display)
		{
			GUI.color = XKCDColors.Beige;
			if (GUILayout.Toggle(scaleIdx == 17, "ΔV", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 17;
			}
		}
		if (graphStates[14].display)
		{
			GUI.color = XKCDColors.Green;
			if (GUILayout.Toggle(scaleIdx == 14, Localizer.Format("#MechJeb_Flightrecord_checkbox29"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 14;
			}
		}
		if (graphStates[15].display)
		{
			GUI.color = XKCDColors.LightBrown;
			if (GUILayout.Toggle(scaleIdx == 15, Localizer.Format("#MechJeb_Flightrecord_checkbox30"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 15;
			}
		}
		if (graphStates[16].display)
		{
			GUI.color = XKCDColors.Cerise;
			if (GUILayout.Toggle(scaleIdx == 16, Localizer.Format("#MechJeb_Flightrecord_checkbox31"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
			{
				scaleIdx = 16;
			}
		}
		GUI.color = color;
		GUILayout.EndVertical();
		GUILayout.EndHorizontal();
		GUILayout.Space(10f);
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		float num5 = (float)((double)width * num4);
		float num6 = Mathf.Max(num5, num);
		if (follow)
		{
			hPos = num6 - num5;
		}
		hPos = GUILayout.HorizontalScrollbar(hPos, num5, 0f, num6, Array.Empty<GUILayoutOption>());
		follow = GUILayout.Toggle(follow, "", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		if ((int)Event.current.type == 7)
		{
			UpdateScale();
			if (graphStates[2].display || graphStates[12].display)
			{
				GUI.DrawTexture(lastRect, (Texture)(object)backgroundTexture, (ScaleMode)0);
			}
			if (stages)
			{
				DrawnStages(lastRect, num4, downrange);
			}
			if (graphStates[2].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.ALTITUDE_ASL, hPos, num4, downrange, XKCDColors.White);
			}
			if (graphStates[12].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.ALTITUDE_TRUE, hPos, num4, downrange, XKCDColors.Grey);
			}
			if (graphStates[7].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.ACCELERATION, hPos, num4, downrange, XKCDColors.LightRed);
			}
			if (graphStates[4].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.SPEED_SURFACE, hPos, num4, downrange, XKCDColors.Yellow);
			}
			if (graphStates[5].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.SPEED_ORBITAL, hPos, num4, downrange, XKCDColors.Apricot);
			}
			if (graphStates[6].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.MASS, hPos, num4, downrange, XKCDColors.Pink);
			}
			if (graphStates[8].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.Q, hPos, num4, downrange, XKCDColors.Cyan);
			}
			if (graphStates[9].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.AO_A, hPos, num4, downrange, XKCDColors.Lavender);
			}
			if (graphStates[10].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.AO_S, hPos, num4, downrange, XKCDColors.Lime);
			}
			if (graphStates[11].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.AO_D, hPos, num4, downrange, XKCDColors.Orange);
			}
			if (graphStates[13].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.PITCH, hPos, num4, downrange, XKCDColors.Mint);
			}
			if (graphStates[17].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.DELTA_V_EXPENDED, hPos, num4, downrange, XKCDColors.Beige);
			}
			if (graphStates[14].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.GRAVITY_LOSSES, hPos, num4, downrange, XKCDColors.Green);
			}
			if (graphStates[15].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.DRAG_LOSSES, hPos, num4, downrange, XKCDColors.LightBrown);
			}
			if (graphStates[16].display)
			{
				DrawnPath(lastRect, MechJebModuleFlightRecorder.RecordType.STEERING_LOSSES, hPos, num4, downrange, XKCDColors.Cerise);
			}
			width = 128 * hSize;
			height = 128 * vSize;
		}
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	private void DrawScaleLabels(Rect r)
	{
		//IL_0083: Unknown result type (might be due to invalid IL or missing references)
		if (scaleIdx == 0)
		{
			return;
		}
		graphState graphState = graphStates[scaleIdx];
		if (graphState.labels != null)
		{
			int labelsActive = graphState.labelsActive;
			double num = (double)height / (graphState.maximum - graphState.minimum);
			float num2 = ((Rect)(ref r)).yMax + (float)(graphState.minimum * num);
			for (int i = 0; i < labelsActive; i++)
			{
				GUI.Label(new Rect(((Rect)(ref r)).xMin - 80f, num2 - (float)(num * graphState.labelsPos[i]) - 10f, 80f, 20f), graphState.labels[i], GuiUtils.MiddleRightLabel);
			}
		}
	}

	private void DrawnPath(Rect r, MechJebModuleFlightRecorder.RecordType type, float minimum, double scaleX, bool downRange, Color color)
	{
		//IL_014c: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ac: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b0: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f0: Unknown result type (might be due to invalid IL or missing references)
		if (recorder.History.Length <= 2 || recorder.HistoryIdx == 0)
		{
			return;
		}
		graphState graphState = graphStates[(int)type];
		double num = (graphState.maximum - graphState.minimum) / (double)height;
		double num2 = 1.0 / scaleX;
		double num3 = 1.0 / num;
		float num4 = (float)((double)((Rect)(ref r)).xMin - (double)minimum * num2);
		float num5 = ((Rect)(ref r)).yMax + (float)(graphState.minimum * num3);
		int i;
		for (i = 0; i < recorder.HistoryIdx && i < recorder.History.Length && num4 + (float)((downRange ? recorder.History[i].DownRange : recorder.History[i].TimeSinceMark) * num2) <= ((Rect)(ref r)).xMin; i++)
		{
		}
		Vector2 val = default(Vector2);
		((Vector2)(ref val))._002Ector(num4 + (float)((downRange ? recorder.History[i].DownRange : recorder.History[i].TimeSinceMark) * num2), num5 - (float)(recorder.History[i][type] * num3));
		Vector2 val2 = default(Vector2);
		for (; i <= recorder.HistoryIdx && i < recorder.History.Length; i++)
		{
			MechJebModuleFlightRecorder.RecordStruct recordStruct = recorder.History[i];
			val2.x = num4 + (float)((downRange ? recordStruct.DownRange : recordStruct.TimeSinceMark) * num2);
			val2.y = num5 - (float)(recordStruct[type] * num3);
			if (((Rect)(ref r)).Contains(val2))
			{
				Vector2 val3 = val - val2;
				if ((double)((Vector2)(ref val3)).sqrMagnitude >= 1.0 || i < 2)
				{
					Drawing.DrawLine(val, val2, color, 2f, true);
					val.x = val2.x;
					val.y = val2.y;
				}
			}
		}
	}

	private void DrawnStages(Rect r, double scaleX, bool downRange)
	{
		//IL_00ae: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bb: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d4: Unknown result type (might be due to invalid IL or missing references)
		if (recorder.History.Length <= 2 || recorder.HistoryIdx == 0)
		{
			return;
		}
		int currentStage = recorder.History[0].CurrentStage;
		Vector2 val = default(Vector2);
		((Vector2)(ref val))._002Ector(0f, ((Rect)(ref r)).yMin);
		Vector2 val2 = default(Vector2);
		((Vector2)(ref val2))._002Ector(0f, ((Rect)(ref r)).yMax);
		for (int i = 1; i <= recorder.HistoryIdx && i < recorder.History.Length; i++)
		{
			MechJebModuleFlightRecorder.RecordStruct recordStruct = recorder.History[i];
			if (recordStruct.CurrentStage != currentStage)
			{
				currentStage = recordStruct.CurrentStage;
				val.x = ((Rect)(ref r)).xMin + (float)((downRange ? recordStruct.DownRange : recordStruct.TimeSinceMark) / scaleX);
				val2.x = val.x;
				if (((Rect)(ref r)).Contains(val))
				{
					Drawing.DrawLine(val, val2, new Color(0.5f, 0.5f, 0.5f), 1f, false);
				}
			}
		}
	}

	private void UpdateScale()
	{
		if (recorder.HistoryIdx == 0)
		{
			ResetScale();
		}
		for (int i = 0; i < typeCount; i++)
		{
			bool flag = false;
			if (graphStates[i].maximum < recorder.Maximums[i])
			{
				flag = true;
				graphStates[i].maximum = recorder.Maximums[i] + Math.Abs(recorder.Maximums[i] * 0.2);
			}
			if (graphStates[i].minimum > recorder.Minimums[i])
			{
				flag = true;
				graphStates[i].minimum = recorder.Minimums[i] - Math.Abs(recorder.Minimums[i] * 0.2);
			}
			if (graphStates[i].labels == null)
			{
				flag = true;
				graphStates[i].labels = new string[11];
				graphStates[i].labelsPos = new double[11];
			}
			if (flag)
			{
				double maximum = graphStates[i].maximum;
				double minimum = graphStates[i].minimum;
				double num = heckbertNiceNum(maximum - minimum, round: false);
				double num2 = heckbertNiceNum(num / 10.0, round: true);
				minimum = Math.Floor(minimum / num2) * num2;
				maximum = Math.Ceiling(maximum / num2) * num2;
				int num3 = (int)Math.Max(0.0 - Math.Floor(Math.Log10(num2)), 0.0);
				double num4 = minimum;
				int num5 = 0;
				while (num4 <= maximum + 0.5 * num2)
				{
					graphStates[i].labels[num5] = num4.ToString("F" + num3);
					graphStates[i].labelsPos[num5] = num4;
					num4 += num2;
					num5++;
				}
				graphStates[i].labelsActive = num5;
				graphStates[i].minimum = minimum;
				graphStates[i].maximum = maximum;
			}
		}
	}

	private void ResetScale()
	{
		graphStates[2].minimum = 0.0;
		graphStates[3].minimum = 0.0;
		graphStates[7].minimum = 0.0;
		graphStates[4].minimum = 0.0;
		graphStates[5].minimum = 0.0;
		graphStates[6].minimum = 0.0;
		graphStates[8].minimum = 0.0;
		graphStates[9].minimum = -5.0;
		graphStates[10].minimum = -5.0;
		graphStates[11].minimum = 0.0;
		graphStates[12].minimum = 0.0;
		graphStates[13].minimum = 0.0;
		graphStates[17].minimum = 0.0;
		graphStates[14].minimum = 0.0;
		graphStates[15].minimum = 0.0;
		graphStates[16].minimum = 0.0;
		graphStates[2].maximum = (((Object)(object)base.MainBody != (Object)null && base.MainBody.atmosphere) ? base.MainBody.RealMaxAtmosphereAltitude() : 10000.0);
		graphStates[3].maximum = 500.0;
		graphStates[7].maximum = 2.0;
		graphStates[4].maximum = 300.0;
		graphStates[5].maximum = 300.0;
		graphStates[6].maximum = 5.0;
		graphStates[8].maximum = 1000.0;
		graphStates[9].maximum = 5.0;
		graphStates[10].maximum = 5.0;
		graphStates[11].maximum = 5.0;
		graphStates[12].maximum = 100.0;
		graphStates[13].maximum = 90.0;
		graphStates[17].maximum = 100.0;
		graphStates[14].maximum = 100.0;
		graphStates[15].maximum = 100.0;
		graphStates[16].maximum = 100.0;
	}

	private double heckbertNiceNum(double x, bool round)
	{
		int num = (int)Math.Log10(x);
		double num2 = x / Math.Pow(10.0, num);
		double num3 = 1.0;
		num3 = (round ? ((num2 < 1.5) ? 1.0 : ((num2 < 3.0) ? 2.0 : ((!(num2 < 7.0)) ? 10.0 : 5.0))) : ((num2 <= 1.0) ? 1.0 : ((num2 <= 2.0) ? 2.0 : ((!(num2 <= 5.0)) ? 10.0 : 5.0))));
		return num3 * Math.Pow(10.0, num);
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(400f),
			GUILayout.Height(300f)
		};
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_Flightrecord_title");
	}

	public override string IconName()
	{
		return "Flight Recorder";
	}
}
