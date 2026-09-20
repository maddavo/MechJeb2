using System;
using System.Collections;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleRoverWindow : DisplayModule
{
	public MechJebModuleRoverController autopilot;

	public MechJebModuleRoverWindow(MechJebCore core)
		: base(core)
	{
	}

	public override void OnStart(StartState state)
	{
		autopilot = Core.GetComputerModule<MechJebModuleRoverController>();
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_Rover_title");
	}

	public override string IconName()
	{
		return "Rover Autopilot";
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(200f),
			GUILayout.Height(50f)
		};
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0572: Unknown result type (might be due to invalid IL or missing references)
		//IL_0633: Unknown result type (might be due to invalid IL or missing references)
		MechJebModuleCustomWindowEditor computerModule = Core.GetComputerModule<MechJebModuleCustomWindowEditor>();
		bool key = GameSettings.MODIFIER_KEY.GetKey(false);
		Rect windowPos = base.WindowPos;
		if (GUI.Button(new Rect(((Rect)(ref windowPos)).width - 48f, 0f, 13f, 20f), "?", GuiUtils.YellowOnHover))
		{
			MechJebModuleWaypointHelpWindow computerModule2 = Core.GetComputerModule<MechJebModuleWaypointHelpWindow>();
			computerModule2.SelTopic = ((IList)computerModule2.Topics).IndexOf((object)"Controller");
			computerModule2.Enabled = computerModule2.SelTopic > -1 || computerModule2.Enabled;
		}
		computerModule.registry.Find((InfoItem i) => i.id == "Toggle:RoverController.ControlHeading").DrawItem();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.heading").DrawItem();
		if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(18f) }))
		{
			autopilot.heading.Val -= ((!GameSettings.MODIFIER_KEY.GetKey(false)) ? 1 : 5);
		}
		if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(18f) }))
		{
			autopilot.heading.Val += ((!GameSettings.MODIFIER_KEY.GetKey(false)) ? 1 : 5);
		}
		GUILayout.EndHorizontal();
		computerModule.registry.Find((InfoItem i) => i.id == "Value:RoverController.headingErr").DrawItem();
		computerModule.registry.Find((InfoItem i) => i.id == "Toggle:RoverController.ControlSpeed").DrawItem();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		computerModule.registry.Find((InfoItem i) => i.id == "Editable:RoverController.speed").DrawItem();
		if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(18f) }))
		{
			autopilot.speed.Val -= ((!GameSettings.MODIFIER_KEY.GetKey(false)) ? 1 : 5);
		}
		if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(18f) }))
		{
			autopilot.speed.Val += ((!GameSettings.MODIFIER_KEY.GetKey(false)) ? 1 : 5);
		}
		GUILayout.EndHorizontal();
		computerModule.registry.Find((InfoItem i) => i.id == "Value:RoverController.speedErr").DrawItem();
		computerModule.registry.Find((InfoItem i) => i.id == "Toggle:RoverController.StabilityControl").DrawItem();
		if (!Core.Settings.HideBrakeOnEject)
		{
			computerModule.registry.Find((InfoItem i) => i.id == "Toggle:RoverController.BrakeOnEject").DrawItem();
		}
		computerModule.registry.Find((InfoItem i) => i.id == "Toggle:RoverController.BrakeOnEnergyDepletion").DrawItem();
		if (autopilot.BrakeOnEnergyDepletion)
		{
			computerModule.registry.Find((InfoItem i) => i.id == "Toggle:RoverController.WarpToDaylight").DrawItem();
		}
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_Rover_label1"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(autopilot.tgtSpeed.ToString("F1"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_Rover_label2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label("Index " + (autopilot.WaypointIndex + 1) + " of " + autopilot.Waypoints.Count, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (Core.Target != null && Core.Target.Target != null)
		{
			Vessel vessel = Core.Target.Target.GetVessel();
			if (GUILayout.Button(Localizer.Format("#MechJeb_Rover_button1"), Array.Empty<GUILayoutOption>()))
			{
				Core.GetComputerModule<MechJebModuleWaypointWindow>().SelIndex = -1;
				autopilot.WaypointIndex = 0;
				autopilot.Waypoints.Clear();
				if ((Object)(object)vessel != (Object)null)
				{
					autopilot.Waypoints.Add(new MechJebWaypoint(vessel, 25f));
				}
				else
				{
					autopilot.Waypoints.Add(new MechJebWaypoint(Core.Target.GetPositionTargetPosition()));
				}
				autopilot.ControlHeading = (autopilot.ControlSpeed = true);
				base.Vessel.ActionGroups.SetGroup((KSPActionGroup)32, false);
				autopilot.LoopWaypoints = key;
			}
			if (GUILayout.Button(Localizer.Format("#MechJeb_Rover_button2"), Array.Empty<GUILayoutOption>()))
			{
				if ((Object)(object)vessel != (Object)null)
				{
					autopilot.Waypoints.Add(new MechJebWaypoint(vessel, 25f));
				}
				else
				{
					autopilot.Waypoints.Add(new MechJebWaypoint(Core.Target.GetPositionTargetPosition()));
				}
			}
		}
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (autopilot.Waypoints.Count > 0)
		{
			if (!autopilot.ControlHeading || !autopilot.ControlSpeed)
			{
				if (GUILayout.Button(Localizer.Format("#MechJeb_Rover_button3"), Array.Empty<GUILayoutOption>()))
				{
					autopilot.WaypointIndex = Mathf.Max(0, (!key) ? autopilot.WaypointIndex : 0);
					autopilot.ControlHeading = (autopilot.ControlSpeed = true);
				}
			}
			else if (GUILayout.Button(Localizer.Format("#MechJeb_Rover_button4"), Array.Empty<GUILayoutOption>()))
			{
				autopilot.ControlHeading = (autopilot.ControlSpeed = (autopilot.LoopWaypoints = false));
			}
		}
		if (GUILayout.Button(Localizer.Format("#MechJeb_Rover_button5"), Array.Empty<GUILayoutOption>()))
		{
			MechJebModuleWaypointWindow computerModule3 = Core.GetComputerModule<MechJebModuleWaypointWindow>();
			computerModule3.Enabled = !computerModule3.Enabled;
			if (computerModule3.Enabled)
			{
				computerModule3.Mode = MechJebModuleWaypointWindow.WaypointMode.ROVER;
			}
		}
		GUILayout.EndHorizontal();
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	public override void OnUpdate()
	{
		if (autopilot != null)
		{
			if (autopilot.ControlHeading || autopilot.ControlSpeed || autopilot.StabilityControl || autopilot.BrakeOnEnergyDepletion || autopilot.BrakeOnEject)
			{
				autopilot.Users.Add(this);
			}
			else
			{
				autopilot.Users.Remove(this);
			}
		}
	}

	protected override void OnModuleDisabled()
	{
		Core.GetComputerModule<MechJebModuleWaypointWindow>().Enabled = false;
		Core.GetComputerModule<MechJebModuleWaypointHelpWindow>().Enabled = false;
		base.OnModuleDisabled();
	}
}
