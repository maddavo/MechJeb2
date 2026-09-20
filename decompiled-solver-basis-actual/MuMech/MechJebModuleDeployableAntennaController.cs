using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleDeployableAntennaController : MechJebModuleDeployableController
{
	public MechJebModuleDeployableAntennaController(MechJebCore core)
		: base(core)
	{
	}

	[GeneralInfoItem("#MechJeb_ToggleAntennas", InfoItem.Category.Misc, showInEditor = false)]
	public void AntennaDeployButton()
	{
		AutoDeploy = GUILayout.Toggle(AutoDeploy, Localizer.Format("#MechJeb_Autodeployantennas"), Array.Empty<GUILayoutOption>());
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
		return p is ModuleDeployableAntenna;
	}

	protected override string GetButtonText(DeployablePartState deployablePartState)
	{
		return deployablePartState switch
		{
			DeployablePartState.EXTENDED => Localizer.Format("#MechJeb_AntennasEXTENDED"), 
			DeployablePartState.RETRACTED => Localizer.Format("#MechJeb_AntennasRETRACTED"), 
			_ => Localizer.Format("#MechJeb_AntennasToggle"), 
		};
	}
}
