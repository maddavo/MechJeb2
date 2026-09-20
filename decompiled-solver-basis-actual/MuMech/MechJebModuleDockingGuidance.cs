using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleDockingGuidance : DisplayModule
{
	private MechJebModuleDockingAutopilot autopilot;

	private static readonly char[] DockingNodeTypeSeparator = new char[1] { ',' };

	private ModuleDockingNode selectedTargetPort;

	private Vessel selectedTargetVessel;

	public MechJebModuleDockingGuidance(MechJebCore core)
		: base(core)
	{
	}

	public override void OnStart(StartState state)
	{
		autopilot = Core.GetComputerModule<MechJebModuleDockingAutopilot>();
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_0108: Unknown result type (might be due to invalid IL or missing references)
		//IL_010d: Unknown result type (might be due to invalid IL or missing references)
		//IL_011d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0122: Unknown result type (might be due to invalid IL or missing references)
		//IL_0508: Unknown result type (might be due to invalid IL or missing references)
		//IL_0513: Unknown result type (might be due to invalid IL or missing references)
		//IL_0518: Unknown result type (might be due to invalid IL or missing references)
		//IL_051d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0529: Unknown result type (might be due to invalid IL or missing references)
		//IL_052e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0539: Unknown result type (might be due to invalid IL or missing references)
		//IL_0545: Unknown result type (might be due to invalid IL or missing references)
		//IL_054a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0561: Unknown result type (might be due to invalid IL or missing references)
		//IL_0566: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Target.NormalTargetExists)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_label1"), Array.Empty<GUILayoutOption>());
			base.WindowGUI(windowID);
			return;
		}
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		if ((Object)(object)base.Vessel.GetReferenceTransformPart() == (Object)null || !base.Vessel.GetReferenceTransformPart().Modules.Contains("ModuleDockingNode"))
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_label2"), GuiUtils.YellowLabel, Array.Empty<GUILayoutOption>());
		}
		DrawTargetPortSelector();
		if (!(Core.Target.Target is ModuleDockingNode) && (Object)(object)selectedTargetPort == (Object)null)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_label3"), GuiUtils.YellowLabel, Array.Empty<GUILayoutOption>());
		}
		bool flag = false;
		foreach (ITargetable item in from t in base.Vessel.GetTargetables()
			where (int)t.GetTargetingMode() == 3
			select t)
		{
			if (Vector3d.Angle(Vector3d.op_Implicit(item.GetTransform().forward), Vector3d.op_Implicit(base.Vessel.ReferenceTransform.up)) < 2.0)
			{
				flag = true;
				break;
			}
		}
		if (!flag)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_label4"), GuiUtils.YellowLabel, Array.Empty<GUILayoutOption>());
		}
		bool flag2 = GUILayout.Toggle(autopilot.Enabled, Localizer.Format("#MechJeb_Docking_checkbox1"), Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_Docking_label5"), autopilot.speedLimit, "m/s");
		autopilot.overrideSafeDistance = GUILayout.Toggle(autopilot.overrideSafeDistance, Localizer.Format("#MechJeb_Docking_checkbox2"), Array.Empty<GUILayoutOption>());
		if (autopilot.overrideSafeDistance)
		{
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_Docking_checkbox3"), autopilot.overridenSafeDistance, "m");
		}
		autopilot.overrideTargetSize = GUILayout.Toggle(autopilot.overrideTargetSize, Localizer.Format("#MechJeb_Docking_checkbox4"), Array.Empty<GUILayoutOption>());
		if (autopilot.overrideTargetSize)
		{
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_Docking_label6"), autopilot.overridenTargetSize, "m");
		}
		if ((double)autopilot.overridenSafeDistance < 0.0)
		{
			autopilot.overridenSafeDistance = 0.0;
		}
		if ((double)autopilot.overridenTargetSize < 10.0)
		{
			autopilot.overridenTargetSize = 10.0;
		}
		autopilot.drawBoundingBox = GUILayout.Toggle(autopilot.drawBoundingBox, Localizer.Format("#MechJeb_Docking_checkbox5"), Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(Localizer.Format("#MechJeb_Docking_button"), Array.Empty<GUILayoutOption>()))
		{
			base.Vessel.GetBoundingBox(debug: true);
			if (Core.Target.Target != null)
			{
				Core.Target.Target.GetVessel().GetBoundingBox(debug: true);
			}
		}
		GUILayout.Label(Localizer.Format("#MechJeb_Docking_label7", new string[1] { autopilot.safeDistance.ToString("F2") }), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.Label(Localizer.Format("#MechJeb_Docking_label8", new string[1] { autopilot.targetSize.ToString("F2") }), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		if ((double)autopilot.speedLimit < 0.0)
		{
			autopilot.speedLimit = 0.0;
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		autopilot.forceRol = GUILayout.Toggle(autopilot.forceRol, Localizer.Format("#MechJeb_Docking_checkbox6"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		autopilot.rol.Text = GUILayout.TextField(autopilot.rol.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(30f) });
		GUILayout.Label("°", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		if (autopilot.Enabled != flag2)
		{
			if (flag2)
			{
				autopilot.Users.Add(this);
			}
			else
			{
				autopilot.Users.Remove(this);
			}
		}
		if (autopilot.Enabled)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_label9", new string[1] { autopilot.status }), Array.Empty<GUILayoutOption>());
			Vector3d val = Core.RCS.targetVelocity - base.VesselState.OrbitalVelocity;
			double num = Vector3d.Dot(val, Vector3d.op_Implicit(base.Vessel.GetTransform().right));
			double num2 = Vector3d.Dot(val, Vector3d.op_Implicit(base.Vessel.GetTransform().forward));
			double num3 = Vector3d.Dot(val, Vector3d.op_Implicit(base.Vessel.GetTransform().up));
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_label10", new string[1] { num.ToString("F2") }) + " m/s  [L/J]", Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_label11", new string[1] { num2.ToString("F2") }) + " m/s  [I/K]", Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_label12", new string[1] { num3.ToString("F2") }) + " m/s  [H/N]", Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_label13", new string[1] { autopilot.zSep.ToString("F2") }) + "m", Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_label14", new string[1] { ((Vector3d)(ref autopilot.lateralSep)).magnitude.ToString("F2") }) + "m", Array.Empty<GUILayoutOption>());
		}
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	private void DrawTargetPortSelector()
	{
		Vessel targetVessel = GetTargetVessel();
		if ((Object)(object)targetVessel == (Object)null || (Object)(object)targetVessel == (Object)(object)base.Vessel)
		{
			selectedTargetPort = null;
			selectedTargetVessel = null;
			return;
		}
		if ((Object)(object)selectedTargetVessel != (Object)(object)targetVessel)
		{
			selectedTargetPort = null;
			selectedTargetVessel = targetVessel;
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_Docking_targetPort"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		ModuleDockingNode referencePort = GetReferenceDockingPort();
		if (!targetVessel.loaded || targetVessel.packed)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_Docking_targetPortUnavailable"), GuiUtils.ArrowSelectorStyeGuiStyleExpand, Array.Empty<GUILayoutOption>());
			GUILayout.EndHorizontal();
			return;
		}
		List<ModuleDockingNode> list = (from port in targetVessel.GetModules<ModuleDockingNode>()
			where IsAvailableTargetPort(port) && (Object)(object)port.GetTransform() != (Object)null && ArePortsCompatible(referencePort, port)
			select port).ToList();
		ITargetable target = Core.Target.Target;
		ModuleDockingNode val = (ModuleDockingNode)(object)((target is ModuleDockingNode) ? target : null);
		if ((Object)(object)val != (Object)null && list.Contains(val))
		{
			selectedTargetPort = val;
		}
		else if ((Object)(object)selectedTargetPort != (Object)null && !list.Contains(selectedTargetPort))
		{
			selectedTargetPort = null;
		}
		if ((Object)(object)val == (Object)null || !list.Contains(val))
		{
			val = selectedTargetPort;
		}
		int num = list.IndexOf(val);
		string text = ((num >= 0) ? Localizer.Format("#MechJeb_Docking_targetPortSelected", new object[3]
		{
			GetPortName(val),
			num + 1,
			list.Count
		}) : ((list.Count <= 0) ? Localizer.Format("#MechJeb_Docking_targetPortNone") : Localizer.Format("#MechJeb_Docking_targetPortSelect", new object[1] { list.Count })));
		bool enabled = GUI.enabled;
		GUI.enabled = enabled && list.Count > 0 && !autopilot.Enabled;
		if (GUILayout.Button("<", (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.ExpandWidth(false) }))
		{
			SelectTargetPort(list, num, -1);
		}
		GUILayout.Label(text, GuiUtils.ArrowSelectorStyeGuiStyleExpand, Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(">", (GUILayoutOption[])(object)new GUILayoutOption[1] { GUILayout.ExpandWidth(false) }))
		{
			SelectTargetPort(list, num, 1);
		}
		GUI.enabled = enabled;
		GUILayout.EndHorizontal();
	}

	private Vessel GetTargetVessel()
	{
		ITargetable target = Core.Target.Target;
		Vessel val = (Vessel)(object)((target is Vessel) ? target : null);
		if (val != null)
		{
			return val;
		}
		ITargetable target2 = Core.Target.Target;
		if (target2 == null)
		{
			return null;
		}
		return target2.GetVessel();
	}

	private ModuleDockingNode GetReferenceDockingPort()
	{
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_007b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		Part referenceTransformPart = base.Vessel.GetReferenceTransformPart();
		if ((Object)(object)referenceTransformPart == (Object)null || (Object)(object)base.Vessel.ReferenceTransform == (Object)null)
		{
			return null;
		}
		ModuleDockingNode result = null;
		double num = 2.0;
		foreach (ModuleDockingNode item in ((IEnumerable)referenceTransformPart.Modules).OfType<ModuleDockingNode>())
		{
			Transform transform = item.GetTransform();
			if (!((Object)(object)transform == (Object)null))
			{
				double num2 = Vector3d.Angle(Vector3d.op_Implicit(transform.forward), Vector3d.op_Implicit(base.Vessel.ReferenceTransform.up));
				if (num2 < num)
				{
					num = num2;
					result = item;
				}
			}
		}
		return result;
	}

	private static bool IsAvailableTargetPort(ModuleDockingNode port)
	{
		if ((Object)(object)port != (Object)null)
		{
			return !IsDockedState(port.state);
		}
		return false;
	}

	private static bool IsDockedState(string state)
	{
		if (state != null)
		{
			if (!state.StartsWith("Docked", StringComparison.Ordinal))
			{
				return state == "PreAttached";
			}
			return true;
		}
		return false;
	}

	private static bool ArePortsCompatible(ModuleDockingNode sourcePort, ModuleDockingNode targetPort)
	{
		if ((Object)(object)sourcePort == (Object)null || (Object)(object)targetPort == (Object)null)
		{
			return false;
		}
		return ArePortsCompatible(sourcePort.nodeType, sourcePort.gendered, sourcePort.genderFemale, targetPort.nodeType, targetPort.gendered, targetPort.genderFemale);
	}

	private static bool ArePortsCompatible(string sourceNodeType, bool sourceGendered, bool sourceFemale, string targetNodeType, bool targetGendered, bool targetFemale)
	{
		if (sourceGendered != targetGendered)
		{
			return false;
		}
		if (sourceGendered && sourceFemale == targetFemale)
		{
			return false;
		}
		if (string.IsNullOrEmpty(sourceNodeType) || string.IsNullOrEmpty(targetNodeType))
		{
			return false;
		}
		string[] first = sourceNodeType.Split(DockingNodeTypeSeparator, StringSplitOptions.RemoveEmptyEntries);
		string[] second = targetNodeType.Split(DockingNodeTypeSeparator, StringSplitOptions.RemoveEmptyEntries);
		return first.Intersect(second).Any();
	}

	private void SelectTargetPort(IReadOnlyList<ModuleDockingNode> targetPorts, int currentIndex, int direction)
	{
		int index = ((currentIndex >= 0) ? ((currentIndex + direction + targetPorts.Count) % targetPorts.Count) : ((direction <= 0) ? (targetPorts.Count - 1) : 0));
		ModuleDockingNode val = (selectedTargetPort = targetPorts[index]);
		Core.Target.Set((ITargetable)(object)val);
		FlightGlobals.fetch.SetVesselTarget((ITargetable)(object)val, false);
	}

	private static string GetPortName(ModuleDockingNode port)
	{
		string name = port.GetName();
		if (!string.IsNullOrEmpty(name))
		{
			return name;
		}
		object obj = ((PartModule)port).part?.partInfo?.title;
		if (obj == null)
		{
			Part part = ((PartModule)port).part;
			obj = ((part != null) ? ((Object)part).name : null) ?? "?";
		}
		return (string)obj;
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(300f),
			GUILayout.Height(50f)
		};
	}

	protected override void OnModuleDisabled()
	{
		if (autopilot != null)
		{
			autopilot.Users.Remove(this);
		}
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_Docking_title");
	}

	public override string IconName()
	{
		return "Docking Autopilot";
	}
}
