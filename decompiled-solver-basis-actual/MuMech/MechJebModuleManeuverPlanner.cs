using System;
using System.Collections.Generic;
using System.Linq;
using KSP.Localization;
using MechJebLibBindings;
using UnityEngine;

namespace MuMech;

public class MechJebModuleManeuverPlanner : DisplayModule
{
	private static readonly Operation[] _operation = Operation.GetAvailableOperations();

	private readonly string[] _operationNames = new List<Operation>(_operation).ConvertAll((Operation x) => x.GetName()).ToArray();

	[Persistent(pass = 4)]
	public int _operationId;

	private bool _createNode = true;

	public MechJebModuleManeuverPlanner(MechJebCore core)
		: base(core)
	{
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_0209: Unknown result type (might be due to invalid IL or missing references)
		_operationId = Mathf.Clamp(_operationId, 0, _operation.Length - 1);
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		List<ManeuverNode> maneuverNodes = GetManeuverNodes();
		bool flag = GetManeuverNodes().Any();
		if (flag)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			if (GUILayout.Button(_createNode ? Localizer.Format("#MechJeb_Maneu_createNodeBtn01") : Localizer.Format("#MechJeb_Maneu_createNodeBtn02"), Array.Empty<GUILayoutOption>()))
			{
				_createNode = !_createNode;
			}
			GUILayout.Label(Localizer.Format("#MechJeb_Maneu_createlab1"), Array.Empty<GUILayoutOption>());
			GUILayout.EndHorizontal();
		}
		else
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Maneu_createlab2"), Array.Empty<GUILayoutOption>());
			_createNode = true;
		}
		_operationId = GuiUtils.ComboBox.Box(_operationId, _operationNames, this);
		double universalTime = base.VesselState.Time;
		Orbit val = base.Orbit;
		if (flag)
		{
			if (_createNode)
			{
				ManeuverNode obj = maneuverNodes.Last();
				universalTime = obj.UT;
				val = obj.nextPatch;
			}
			else if (maneuverNodes.Count > 1)
			{
				ManeuverNode obj2 = maneuverNodes[maneuverNodes.Count - 1];
				universalTime = obj2.UT;
				val = obj2.nextPatch;
			}
		}
		try
		{
			_operation[_operationId].DoParametersGUI(val, universalTime, Core.Target);
		}
		catch (Exception)
		{
		}
		if (flag)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Maneu_createlab3"), Array.Empty<GUILayoutOption>());
		}
		bool flag2 = false;
		bool flag3 = false;
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(Localizer.Format("#MechJeb_Maneu_button1"), Array.Empty<GUILayoutOption>()))
		{
			flag2 = true;
		}
		if (Core.Node != null && GUILayout.Button(Localizer.Format("#MechJeb_Maneu_button2"), Array.Empty<GUILayoutOption>()))
		{
			flag2 = true;
			flag3 = true;
		}
		GUILayout.EndHorizontal();
		if (flag2)
		{
			List<ManeuverParameters> list = _operation[_operationId].MakeNodes(val, universalTime, Core.Target);
			if (list != null)
			{
				if (!_createNode)
				{
					maneuverNodes.Last().RemoveSelf();
				}
				for (int i = 0; i < list.Count; i++)
				{
					base.Vessel.PlaceManeuverNode(val, list[i].dV, list[i].UT);
				}
			}
			if (flag3 && Core.Node != null)
			{
				Core.Node.ExecuteOneNode(this);
			}
		}
		if (_operation[_operationId].GetErrorMessage().Length > 0)
		{
			GUILayout.Label(_operation[_operationId].GetErrorMessage(), GuiUtils.YellowLabel, Array.Empty<GUILayoutOption>());
		}
		if (GUILayout.Button(Localizer.Format("#MechJeb_Maneu_button3"), Array.Empty<GUILayoutOption>()))
		{
			base.Vessel.RemoveAllManeuverNodes();
		}
		if (Core.Node != null)
		{
			if (flag && !Core.Node.Enabled)
			{
				if (GUILayout.Button(Localizer.Format("#MechJeb_Maneu_button4"), Array.Empty<GUILayoutOption>()))
				{
					Core.Node.ExecuteOneNode(this);
				}
				if ((base.Vessel.patchedConicSolver.maneuverNodes.Count > 1 || ReflectionUtils.IsLoadedPrincipia) && GUILayout.Button(Localizer.Format("#MechJeb_Maneu_button5"), Array.Empty<GUILayoutOption>()))
				{
					Core.Node.ExecuteAllNodes(this);
				}
			}
			else if (Core.Node.Enabled && GUILayout.Button(Localizer.Format("#MechJeb_Maneu_button6"), Array.Empty<GUILayoutOption>()))
			{
				Core.Node.Abort();
			}
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			Core.Node.Autowarp = GUILayout.Toggle(Core.Node.Autowarp, Localizer.Format("#MechJeb_Maneu_Autowarp"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			Core.Node.RCSOnly = GUILayout.Toggle(Core.Node.RCSOnly, "RCS Burn", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			Core.Node.KillRollRotation = GUILayout.Toggle(Core.Node.KillRollRotation, "Kill Rotation", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_Maneu_Lead_time"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			Core.Node.LeadTime.Text = GUILayout.TextField(Core.Node.LeadTime.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
			{
				GuiUtils.LayoutWidth(35f),
				GuiUtils.LayoutNoExpandWidth
			});
			GUILayout.Label("s", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
			{
				Core.Node.LeadTime.Val += 1.0;
			}
			if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
			{
				Core.Node.LeadTime.Val -= 1.0;
			}
			if (GUILayout.Button("R", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
			{
				Core.Node.LeadTime.Val = 3.0;
			}
			GUILayout.EndHorizontal();
		}
		GUILayout.EndVertical();
		WindowGUI(windowID, _operation[_operationId].Draggable);
	}

	public List<ManeuverNode> GetManeuverNodes()
	{
		MechJebModuleLandingPredictions predictor = Core.GetComputerModule<MechJebModuleLandingPredictions>();
		if (predictor == null)
		{
			return base.Vessel.patchedConicSolver.maneuverNodes;
		}
		return base.Vessel.patchedConicSolver.maneuverNodes.Where((ManeuverNode n) => n != predictor.aerobrakeNode).ToList();
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
		return Localizer.Format("#MechJeb_Maneuver_Planner_title");
	}

	public override string IconName()
	{
		return "Maneuver Planner";
	}

	protected override bool IsSpaceCenterUpgradeUnlocked()
	{
		return base.Vessel.patchedConicsUnlocked();
	}
}
