using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleRendezvousAutopilotWindow : DisplayModule
{
	public MechJebModuleRendezvousAutopilotWindow(MechJebCore core)
		: base(core)
	{
	}

	protected override void WindowGUI(int windowID)
	{
		if (!Core.Target.NormalTargetExists)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_RZauto_label1"), Array.Empty<GUILayoutOption>());
			base.WindowGUI(windowID);
			return;
		}
		MechJebModuleRendezvousAutopilot computerModule = Core.GetComputerModule<MechJebModuleRendezvousAutopilot>();
		if ((Object)(object)Core.Target.TargetOrbit.referenceBody != (Object)(object)base.Orbit.referenceBody)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_RZauto_label2"), Array.Empty<GUILayoutOption>());
			if (computerModule.Enabled)
			{
				computerModule.Users.Remove(this);
			}
			base.WindowGUI(windowID);
			return;
		}
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		if (computerModule != null)
		{
			GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_RZauto_label3"), Core.Target.Name);
			if (!computerModule.Enabled)
			{
				if (GUILayout.Button(Localizer.Format("#MechJeb_RZauto_button1"), Array.Empty<GUILayoutOption>()))
				{
					computerModule.Users.Add(this);
				}
			}
			else if (GUILayout.Button(Localizer.Format("#MechJeb_RZauto_button2"), Array.Empty<GUILayoutOption>()))
			{
				computerModule.Users.Remove(this);
			}
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RZauto_label4"), computerModule.desiredDistance, "m");
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RZauto_label5"), computerModule.maxPhasingOrbits);
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RZauto_label8"), computerModule.maxClosingSpeed, "m/s");
			if ((double)computerModule.maxPhasingOrbits < 5.0)
			{
				GUILayout.Label(Localizer.Format("#MechJeb_RZauto_label6"), GuiUtils.YellowLabel, Array.Empty<GUILayoutOption>());
			}
			if (computerModule.Enabled)
			{
				GUILayout.Label(Localizer.Format("#MechJeb_RZauto_label7", new string[1] { computerModule.status }), Array.Empty<GUILayoutOption>());
			}
		}
		Core.Node.Autowarp = GUILayout.Toggle(Core.Node.Autowarp, Localizer.Format("#MechJeb_RZauto_checkbox1"), Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(300f),
			GUILayout.Height(50f)
		};
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_RZauto_title");
	}

	public override string IconName()
	{
		return "Rendezvous Autopilot";
	}

	protected override bool IsSpaceCenterUpgradeUnlocked()
	{
		return base.Vessel.patchedConicsUnlocked();
	}
}
