using System;
using System.Linq;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleRendezvousGuidance : DisplayModule
{
	private readonly EditableDoubleMult phasingOrbitAltitude = new EditableDoubleMult(200000.0, 1000.0);

	public MechJebModuleRendezvousGuidance(MechJebCore core)
		: base(core)
	{
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_0290: Unknown result type (might be due to invalid IL or missing references)
		//IL_0295: Unknown result type (might be due to invalid IL or missing references)
		//IL_0265: Unknown result type (might be due to invalid IL or missing references)
		//IL_026a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0501: Unknown result type (might be due to invalid IL or missing references)
		//IL_0506: Unknown result type (might be due to invalid IL or missing references)
		//IL_0526: Unknown result type (might be due to invalid IL or missing references)
		//IL_032f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0334: Unknown result type (might be due to invalid IL or missing references)
		//IL_0342: Unknown result type (might be due to invalid IL or missing references)
		//IL_0378: Unknown result type (might be due to invalid IL or missing references)
		//IL_037d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0387: Unknown result type (might be due to invalid IL or missing references)
		//IL_02ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_0561: Unknown result type (might be due to invalid IL or missing references)
		//IL_0566: Unknown result type (might be due to invalid IL or missing references)
		//IL_057f: Unknown result type (might be due to invalid IL or missing references)
		//IL_044f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0454: Unknown result type (might be due to invalid IL or missing references)
		//IL_0462: Unknown result type (might be due to invalid IL or missing references)
		//IL_03c9: Unknown result type (might be due to invalid IL or missing references)
		//IL_03ce: Unknown result type (might be due to invalid IL or missing references)
		//IL_03dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0412: Unknown result type (might be due to invalid IL or missing references)
		//IL_0417: Unknown result type (might be due to invalid IL or missing references)
		//IL_0421: Unknown result type (might be due to invalid IL or missing references)
		//IL_05dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_05e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_05fa: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Target.NormalTargetExists)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_RZplan_label1"), Array.Empty<GUILayoutOption>());
			base.WindowGUI(windowID);
			return;
		}
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_RZplan_label2"), Array.Empty<GUILayoutOption>());
			base.WindowGUI(windowID);
			return;
		}
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RZplan_label3"), Core.Target.Name);
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RZplan_label4"), Statics.ToSI(Core.Target.TargetOrbit.PeA, 3, int.MaxValue) + "m x " + Statics.ToSI(Core.Target.TargetOrbit.ApA, 3, int.MaxValue) + "m");
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RZplan_label5"), Statics.ToSI(base.Orbit.PeA, 3, int.MaxValue) + "m x " + Statics.ToSI(base.Orbit.ApA, 3, int.MaxValue) + "m");
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RZplan_label6"), base.Orbit.RelativeInclination(Core.Target.TargetOrbit).ToString("F2") + "º");
		double num = base.Orbit.NextClosestApproachTime(Core.Target.TargetOrbit, base.VesselState.Time);
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RZplan_label7"), GuiUtils.TimeToDHMS(num - base.VesselState.Time));
		GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RZplan_label8"), Statics.ToSI(base.Orbit.Separation(Core.Target.TargetOrbit, num), 4, int.MaxValue) + "m");
		if (GUILayout.Button(Localizer.Format("#MechJeb_RZplan_button1"), Array.Empty<GUILayoutOption>()))
		{
			double burnUT;
			Vector3d dV = ((!base.Orbit.AscendingNodeExists(Core.Target.TargetOrbit)) ? OrbitalManeuverCalculator.DeltaVAndTimeToMatchPlanesDescending(base.Orbit, Core.Target.TargetOrbit, base.VesselState.Time, out burnUT) : OrbitalManeuverCalculator.DeltaVAndTimeToMatchPlanesAscending(base.Orbit, Core.Target.TargetOrbit, base.VesselState.Time, out burnUT));
			base.Vessel.RemoveAllManeuverNodes();
			base.Vessel.PlaceManeuverNode(base.Orbit, dV, burnUT);
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(Localizer.Format("#MechJeb_RZplan_button2"), Array.Empty<GUILayoutOption>()))
		{
			double num2 = (double)phasingOrbitAltitude + base.MainBody.Radius;
			base.Vessel.RemoveAllManeuverNodes();
			if (base.Orbit.ApR < num2)
			{
				double num3 = base.VesselState.Time + 30.0;
				Vector3d dV2 = OrbitalManeuverCalculator.DeltaVToChangeApoapsis(base.Orbit, num3, num2);
				base.Vessel.PlaceManeuverNode(base.Orbit, dV2, num3);
				Orbit nextPatch = base.Vessel.patchedConicSolver.maneuverNodes[0].nextPatch;
				double num4 = nextPatch.NextApoapsisTime(num3);
				Vector3d dV3 = OrbitalManeuverCalculator.DeltaVToCircularize(nextPatch, num4);
				base.Vessel.PlaceManeuverNode(nextPatch, dV3, num4);
			}
			else if (base.Orbit.PeR > num2)
			{
				double num5 = base.VesselState.Time + 30.0;
				Vector3d dV4 = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(base.Orbit, num5, num2);
				base.Vessel.PlaceManeuverNode(base.Orbit, dV4, num5);
				Orbit nextPatch2 = base.Vessel.patchedConicSolver.maneuverNodes[0].nextPatch;
				double num6 = nextPatch2.NextPeriapsisTime(num5);
				Vector3d dV5 = OrbitalManeuverCalculator.DeltaVToCircularize(nextPatch2, num6);
				base.Vessel.PlaceManeuverNode(nextPatch2, dV5, num6);
			}
			else
			{
				double num7 = base.Orbit.NextTimeOfRadius(base.VesselState.Time, num2);
				Vector3d dV6 = OrbitalManeuverCalculator.DeltaVToCircularize(base.Orbit, num7);
				base.Vessel.PlaceManeuverNode(base.Orbit, dV6, num7);
			}
		}
		phasingOrbitAltitude.Text = GUILayout.TextField(phasingOrbitAltitude.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(70f) });
		GUILayout.Label("km", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		if (GUILayout.Button(Localizer.Format("#MechJeb_RZplan_button3"), Array.Empty<GUILayoutOption>()))
		{
			var (dV7, uT, _, _) = OrbitalManeuverCalculator.DeltaVAndTimeForHohmannTransfer(base.Orbit, Core.Target.TargetOrbit, base.VesselState.Time, double.NaN, fixedTime: false, coplanar: false);
			base.Vessel.RemoveAllManeuverNodes();
			base.Vessel.PlaceManeuverNode(base.Orbit, dV7, uT);
		}
		if (GUILayout.Button(Localizer.Format("#MechJeb_RZplan_button4"), Array.Empty<GUILayoutOption>()))
		{
			double uT2 = num;
			Vector3d dV8 = OrbitalManeuverCalculator.DeltaVToMatchVelocities(base.Orbit, uT2, Core.Target.TargetOrbit);
			base.Vessel.RemoveAllManeuverNodes();
			base.Vessel.PlaceManeuverNode(base.Orbit, dV8, uT2);
		}
		if (GUILayout.Button(Localizer.Format("#MechJeb_RZplan_button5"), Array.Empty<GUILayoutOption>()))
		{
			double time = base.VesselState.Time;
			Vector3d item = OrbitalManeuverCalculator.DeltaVToInterceptAtTime(base.Orbit, time, Core.Target.TargetOrbit, 100.0, 10.0).v1;
			base.Vessel.RemoveAllManeuverNodes();
			base.Vessel.PlaceManeuverNode(base.Orbit, item, time);
		}
		if (GUILayout.Button(Localizer.Format("#MechJeb_RZplan_button9"), Array.Empty<GUILayoutOption>()))
		{
			base.Vessel.RemoveAllManeuverNodes();
		}
		if (Core.Node != null)
		{
			if (base.Vessel.patchedConicSolver.maneuverNodes.Any() && !Core.Node.Enabled)
			{
				if (GUILayout.Button(Localizer.Format("#MechJeb_RZplan_button6"), Array.Empty<GUILayoutOption>()))
				{
					Core.Node.ExecuteOneNode(this);
				}
				if (base.Vessel.patchedConicSolver.maneuverNodes.Count > 1 && GUILayout.Button(Localizer.Format("#MechJeb_RZplan_button7"), Array.Empty<GUILayoutOption>()))
				{
					Core.Node.ExecuteAllNodes(this);
				}
			}
			else if (Core.Node.Enabled && GUILayout.Button(Localizer.Format("#MechJeb_RZplan_button8"), Array.Empty<GUILayoutOption>()))
			{
				Core.Node.Abort();
			}
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			Core.Node.Autowarp = GUILayout.Toggle(Core.Node.Autowarp, Localizer.Format("#MechJeb_RZplan_checkbox"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.EndHorizontal();
		}
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(300f),
			GUILayout.Height(150f)
		};
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_RZplan_title");
	}

	public override string IconName()
	{
		return "Rendezvous Planner";
	}

	protected override bool IsSpaceCenterUpgradeUnlocked()
	{
		return base.Vessel.patchedConicsUnlocked();
	}
}
