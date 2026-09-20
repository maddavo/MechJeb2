using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleSolarPanelController : MechJebModuleDeployableController
{
	public MechJebModuleSolarPanelController(MechJebCore core)
		: base(core)
	{
	}

	[GeneralInfoItem("#MechJeb_ToggleSolarPanels", InfoItem.Category.Misc, showInEditor = false)]
	public void SolarPanelDeployButton()
	{
		AutoDeploy = GUILayout.Toggle(AutoDeploy, Localizer.Format("#MechJeb_SolarPanelDeployButton"), Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(ButtonText, Array.Empty<GUILayoutOption>()) && !ExtendingOrRetracting())
		{
			if (!Extended)
			{
				ExtendAll();
			}
			else
			{
				RetractAll();
			}
		}
	}

	protected override bool IsModules(ModuleDeployablePart p)
	{
		return p is ModuleDeployableSolarPanel;
	}

	protected override string GetButtonText(DeployablePartState deployablePartState)
	{
		return deployablePartState switch
		{
			DeployablePartState.EXTENDED => Localizer.Format("#MechJeb_SolarPanelDeploy"), 
			DeployablePartState.RETRACTED => Localizer.Format("#MechJeb_SolarPanelRetracted"), 
			_ => Localizer.Format("#MechJeb_SolarPanelToggle"), 
		};
	}
}
