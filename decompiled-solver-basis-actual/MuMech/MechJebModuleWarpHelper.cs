using System;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleWarpHelper : DisplayModule
{
	public enum WarpTarget
	{
		Periapsis,
		Apoapsis,
		Node,
		SoI,
		Time,
		PhaseAngleT,
		HoverslamBurn,
		AtmosphericEntry
	}

	private static readonly string[] warpTargetStrings = new string[8]
	{
		Localizer.Format("#MechJeb_WarpHelper_Combobox_text1"),
		Localizer.Format("#MechJeb_WarpHelper_Combobox_text2"),
		Localizer.Format("#MechJeb_WarpHelper_Combobox_text3"),
		Localizer.Format("#MechJeb_WarpHelper_Combobox_text4"),
		Localizer.Format("#MechJeb_WarpHelper_Combobox_text5"),
		Localizer.Format("#MechJeb_WarpHelper_Combobox_text6"),
		Localizer.Format("#MechJeb_WarpHelper_Combobox_text7"),
		Localizer.Format("#MechJeb_WarpHelper_Combobox_text8")
	};

	[Persistent(pass = 4)]
	public WarpTarget warpTarget;

	[Persistent(pass = 4)]
	public readonly EditableTime leadTime = 0.0;

	public bool warping;

	public readonly EditableTime timeOffset = 0.0;

	private double targetUT;

	[Persistent(pass = 7)]
	public readonly EditableDouble phaseAngle = 0.0;

	protected override void WindowGUI(int windowID)
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_WarpHelper_label1"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		warpTarget = (WarpTarget)GuiUtils.ComboBox.Box((int)warpTarget, warpTargetStrings, this);
		GUILayout.EndHorizontal();
		if (warpTarget == WarpTarget.Time)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_WarpHelper_label2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			timeOffset.Text = GUILayout.TextField(timeOffset.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(100f) });
			GUILayout.EndHorizontal();
		}
		else if (warpTarget == WarpTarget.PhaseAngleT)
		{
			if (!Core.Target.NormalTargetExists)
			{
				GUILayout.Label(Localizer.Format("#MechJeb_WarpHelper_label3"), Array.Empty<GUILayoutOption>());
			}
			else
			{
				GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_WarpHelper_label4"), phaseAngle, "º", 60f);
			}
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_WarpHelper_label5"), leadTime, "");
		if (warping)
		{
			if (GUILayout.Button(Localizer.Format("#MechJeb_WarpHelper_button1"), Array.Empty<GUILayoutOption>()))
			{
				AbortWarp();
			}
		}
		else if (GUILayout.Button(Localizer.Format("#MechJeb_WarpHelper_button2"), Array.Empty<GUILayoutOption>()))
		{
			StartWarp();
		}
		GUILayout.EndHorizontal();
		Core.Warp.useQuickWarpInfoItem();
		if (warping)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_WarpHelper_label6") + (((double)leadTime > 0.0) ? (GuiUtils.TimeToDHMS(leadTime) + " before ") : "") + warpTargetStrings[(int)warpTarget] + ".", Array.Empty<GUILayoutOption>());
		}
		Core.Warp.ControlWarpButton();
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	public void StartWarp()
	{
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Invalid comparison between Unknown and I4
		warping = true;
		switch (warpTarget)
		{
		case WarpTarget.Periapsis:
			targetUT = base.Orbit.NextPeriapsisTime(base.VesselState.Time);
			break;
		case WarpTarget.Apoapsis:
			if (base.Orbit.eccentricity < 1.0)
			{
				targetUT = base.Orbit.NextApoapsisTime(base.VesselState.Time);
			}
			break;
		case WarpTarget.SoI:
			if ((int)base.Orbit.patchEndTransition != 1)
			{
				targetUT = base.Orbit.EndUT;
			}
			break;
		case WarpTarget.Node:
			if (base.Vessel.patchedConicsUnlocked() && base.Vessel.patchedConicSolver.maneuverNodes.Any())
			{
				targetUT = base.Vessel.patchedConicSolver.maneuverNodes[0].UT;
			}
			break;
		case WarpTarget.Time:
			targetUT = base.VesselState.Time + (double)timeOffset;
			break;
		case WarpTarget.PhaseAngleT:
			if (Core.Target.NormalTargetExists)
			{
				Orbit val = ((!((Object)(object)Core.Target.TargetOrbit.referenceBody == (Object)(object)base.Orbit.referenceBody)) ? base.Orbit.referenceBody.orbit : base.Orbit);
				double num = 360.0 / Core.Target.TargetOrbit.period - 360.0 / val.period;
				double num2 = val.PhaseAngle(Core.Target.TargetOrbit, base.VesselState.Time) - (double)phaseAngle;
				if (num2 > 0.0 && num > 0.0)
				{
					num2 -= 360.0;
				}
				if (num2 < 0.0 && num < 0.0)
				{
					num2 += 360.0;
				}
				double num3 = Math.Floor(Math.Abs(num2 / num));
				targetUT = base.VesselState.Time + num3;
			}
			break;
		case WarpTarget.AtmosphericEntry:
			try
			{
				targetUT = base.Vessel.orbit.NextTimeOfRadius(base.VesselState.Time, base.VesselState.MainBody.Radius + base.VesselState.MainBody.RealMaxAtmosphereAltitude());
				break;
			}
			catch
			{
				warping = false;
				break;
			}
		case WarpTarget.HoverslamBurn:
			try
			{
				targetUT = Core.GetComputerModule<MechJebModuleHoverslamSimulation>().IgnitionUT;
				break;
			}
			catch
			{
				warping = false;
				break;
			}
		default:
			targetUT = base.VesselState.Time;
			break;
		}
	}

	public void AbortWarp()
	{
		warping = false;
		Core.Warp.MinimumWarp(instant: true);
	}

	public override void OnFixedUpdate()
	{
		if (!warping)
		{
			return;
		}
		if (warpTarget == WarpTarget.HoverslamBurn)
		{
			try
			{
				targetUT = Core.GetComputerModule<MechJebModuleHoverslamSimulation>().IgnitionUT;
			}
			catch
			{
				warping = false;
			}
		}
		double num = targetUT - (double)leadTime;
		if (num < base.VesselState.Time + 1.0)
		{
			Core.Warp.MinimumWarp(instant: true);
			warping = false;
		}
		else
		{
			Core.Warp.WarpToUT(num);
		}
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(240f),
			GUILayout.Height(50f)
		};
	}

	public override bool IsActive()
	{
		return warping;
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_WarpHelper_title");
	}

	public override string IconName()
	{
		return "Warp Helper";
	}

	public MechJebModuleWarpHelper(MechJebCore core)
		: base(core)
	{
	}
}
