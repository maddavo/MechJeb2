using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleWaypointHelpWindow : DisplayModule
{
	public int SelTopic;

	public readonly string[] Topics = new string[4] { "Rover Controller", "Waypoints", "Routes", "Settings" };

	private string _selSubTopic = "";

	private GUIStyle _btnActive;

	private GUIStyle _btnInactive;

	private void HelpTopic(string title, string text)
	{
		//IL_0051: Unknown result type (might be due to invalid IL or missing references)
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0072: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(title, (_selSubTopic == title) ? _btnActive : _btnInactive, Array.Empty<GUILayoutOption>()))
		{
			_selSubTopic = ((_selSubTopic != title) ? title : "");
			Rect windowPos = base.WindowPos;
			float x = ((Rect)(ref windowPos)).x;
			windowPos = base.WindowPos;
			float y = ((Rect)(ref windowPos)).y;
			windowPos = base.WindowPos;
			base.WindowPos = new Rect(x, y, ((Rect)(ref windowPos)).width, 0f);
		}
		if (_selSubTopic == title)
		{
			GUILayout.Label(text, Array.Empty<GUILayoutOption>());
		}
		GUILayout.EndVertical();
	}

	public MechJebModuleWaypointHelpWindow(MechJebCore core)
		: base(core)
	{
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_Waypointhelper_title");
	}

	public override string IconName()
	{
		return "Waypoint Help";
	}

	public override void OnStart(StartState state)
	{
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		Hidden = true;
		base.OnStart(state);
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Expected O, but got Unknown
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003d: Expected O, but got Unknown
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_0076: Unknown result type (might be due to invalid IL or missing references)
		//IL_0077: Unknown result type (might be due to invalid IL or missing references)
		//IL_007d: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		if (_btnInactive == null)
		{
			_btnInactive = new GUIStyle(GuiUtils.Skin.button)
			{
				alignment = (TextAnchor)3
			};
		}
		if (_btnActive == null)
		{
			_btnActive = new GUIStyle(_btnInactive);
			GUIStyleState active = _btnActive.active;
			GUIStyleState hover = _btnActive.hover;
			GUIStyleState focused = _btnActive.focused;
			Color val = (_btnActive.normal.textColor = Color.green);
			Color val3 = (focused.textColor = val);
			Color textColor = (hover.textColor = val3);
			active.textColor = textColor;
		}
		SelTopic = GUILayout.SelectionGrid(SelTopic, Topics, Topics.Length, Array.Empty<GUILayoutOption>());
		switch (Topics[SelTopic])
		{
		case "Rover Controller":
			HelpTopic("Holding a set Heading", "To hold a specific heading just tick the box next to 'Heading control' and the autopilot will try to keep going for the entered heading.\nThis also needs to be enabled when the autopilot is supposed to drive to a waypoint'Heading Error' simply shows the error between current heading and target heading.");
			HelpTopic("Holding a set Speed", "To hold a specific speed just tick the box next to 'Speed control' and the autopilot will try to keep going at the entered speed.\nThis also needs to be enabled when the autopilot is supposed to drive to a waypoint'Speed Error' simply shows the error between current speed and target speed.");
			HelpTopic("More stability while driving and nice landings", "If you turn on 'Stability Control' then the autopilot will use the reaction wheel's torque to keep the rover aligned with the surface.\nThis means that if you make a jump the autopilot will try to align the rover in the best possible way to land as straight and flat as possible given the available time and torque.\nBe aware that this doesn't make your rover indestructible, only relatively unlikely to land in a bad way.\n\n'Stability Control' will also limit the brake to reduce the chances of flipping over from too much braking power.\nSee 'Settings' -> 'Traction and Braking'. This setting is also saved per vessel.");
			HelpTopic("Brake on Pilot Eject", "With this option enabled the rover will stop if the pilot (on manned rovers) should get thrown out of his seat.");
			HelpTopic("Target Speed", "Current speed the autopilot tries to achieve.");
			HelpTopic("Waypoint Index", "Overview of waypoints and which the autopilot is currently driving to.");
			HelpTopic("Button 'Waypoints'", "Opens the waypoint list to set up a route.");
			HelpTopic("Button 'Follow' / 'Stop'", "This sets the autopilot to drive along the set route starting at the first waypoint. Only visible when atleast one waypoint is set.\n\nAlt click will set the autopilot to 'Loop Mode' which will make it go for the first waypoint again after reaching the last.If the only waypoint happens to be a target it will keep following that instead of only going to it once.\n\nIf the autopilot is already active the 'Follow' button will turn into the 'Stop' button which will obviously stop it when pressed.");
			HelpTopic("Button 'To Target'", "Clears the route, adds the target as only waypoint and starts the autopilot. Only visible with a selected target.\n\nAlt click will set the autopilot to 'Loop Mode' which will make it continue to follow the target, pausing when near it instead of turning off then.");
			HelpTopic("Button 'Add Target'", "Adds the selected target as a waypoint either at the end of the route or before the selected waypoint. Only visible with a selected target.");
			break;
		case "Waypoints":
			HelpTopic("Adding Waypoints", "Adds a new waypoint to the route at the end or before the currently selected waypoint, simply click the terrain or somewhere on the body in Mapview.\n\nAlt clicking will reverse the route for easier going back and holding Alt while clicking the terrain or body in Mapview will allow to add more waypoints without having to click the button again.");
			HelpTopic("Removing Waypoints", "Removes the currently selected waypoint.\n\nAlt clicking will remove all waypoints.");
			HelpTopic("Reordering Waypoints", "'Up' and 'Down' will move the selected waypoint up or down in the list, Alt clicking will move it to the top or bottom respectively.");
			HelpTopic("Waypoint Radius", "Radius defines the distance to the center of the waypoint after which the waypoint will be considered 'reached'.\n\nA radius of 5m (default) simply means that when you're 5m from the waypoint away the autopilot will jump to the next or turn off if it was the last.\n\nThe 'A' button behind the textfield will set the entered radius for all waypoints.");
			HelpTopic("Speedlimits", "The two speed textfields represent the minimum and maximum speed for the waypoint.\n\nThe maximum speed is the speed the autopilot tries to reach to get to the waypoint.\n\nThe minimum speed was before used to set the speed with which the autopilot will go through the waypoint, but that got reworked now to be based on the next waypoint's max. speed and the turn needed at the waypoint.\n\nI have no idea what this will currently do if set so better just leave it at 0...\n\nThe 'A' buttons set their respective speed for all waypoints.");
			HelpTopic("Quicksaving at a Waypoint", "Clicking the 'QS' button will turn on QuickSave for that waypoint.\n\nThis will make the autopilot stop and try to quicksave at that waypoint and then continue. A QuickSave waypoint has yellow text instead of white.\n\nSmall sideeffect: leaving the throttle up will prevent the saving from occurring effectively pausing the autopilot at that point until interefered with. (Discovered by Greys)\n\nAlt clicking will toggle QS for all waypoints including the clicked one.");
			HelpTopic("Changing the current target Waypoint", "Alt clicking a waypoint will mark it as the current target waypoint. The active waypoint has a green tinted background.");
			break;
		case "Routes":
			HelpTopic("Routes Help", "The empty textfield is for saving routes, enter a name there before clicking 'Save'.\n\nTo load a route simply select one from the list and click 'Load'.\n\nTo delete a route simply select it and a 'Delete' button will appear right of it.");
			break;
		case "Settings":
			HelpTopic("Heading / Speed PID", "These parameters control the behaviour of the heading's / speed's PID. Saved globally so NO TOUCHING unless you know what you're doing (or atleast know how to write down numbers to restore it if you mess up)");
			HelpTopic("Safe Turn Speed", "'Safe Turn Speed' tells the autopilot which speed the rover can usually go full turn through corners without tipping over.\n\nGiven how differently terrain can be and other influences you can just leave it at 3 m/s but if you're impatient or just want to experiment feel free to test around. Saved per vessel type (same named vessels will share the setting).");
			HelpTopic("Traction and Braking", "'Traction' shows in % how many wheels have ground contact.\n'Traction Brake Limit' defines what traction is atleast needed for the autopilot to still apply the brakes (given 'Stability Control' is active) even if you hold the brake down.\nThis means the default setting of 75 will make it brake only if atleast 3 wheels have ground contact.\n'Traction Brake Limit' is saved per vessel type.\n\nIf you have 'Stability Control' off then it won't take care of your brake and you can flip as much as you want.");
			HelpTopic("Changing the route height in Mapview", "These values define offsets for the route height in Mapview. Given how weird it's set up it can be that they are too high or too low so I added these for easier adjusting. Saved globally, I think.");
			break;
		}
		base.WindowGUI(windowID);
	}
}
