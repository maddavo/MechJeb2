using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleNodeEditor : DisplayModule
{
	private enum Snap
	{
		PERIAPSIS,
		APOAPSIS,
		REL_ASCENDING,
		REL_DESCENDING,
		EQ_ASCENDING,
		EQ_DESCENDING
	}

	private EditableDouble prograde = 0.0;

	private EditableDouble radialPlus = 0.0;

	private EditableDouble normalPlus = 0.0;

	[Persistent(pass = 4)]
	public EditableDouble progradeDelta = 0.0;

	[Persistent(pass = 4)]
	public EditableDouble radialPlusDelta = 0.0;

	[Persistent(pass = 4)]
	public EditableDouble normalPlusDelta = 0.0;

	[Persistent(pass = 4)]
	public readonly EditableTime timeOffset = 0.0;

	private ManeuverNode node;

	private ManeuverGizmo gizmo;

	private static readonly int numSnaps = Enum.GetNames(typeof(Snap)).Length;

	private Snap snap;

	private readonly string[] snapStrings = new string[6]
	{
		Localizer.Format("#MechJeb_NodeEd_Snap1"),
		Localizer.Format("#MechJeb_NodeEd_Snap2"),
		Localizer.Format("#MechJeb_NodeEd_Snap3"),
		Localizer.Format("#MechJeb_NodeEd_Snap4"),
		Localizer.Format("#MechJeb_NodeEd_Snap5"),
		Localizer.Format("#MechJeb_NodeEd_Snap6")
	};

	private static float nextClick;

	private static readonly string[] relativityModeStrings = new string[5] { "0", "1", "2", "3", "4" };

	private void GizmoUpdateHandler(Vector3d dV, double UT)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		prograde = dV.z;
		radialPlus = dV.x;
		normalPlus = dV.y;
	}

	private void MergeNext(int index)
	{
		//IL_0051: Unknown result type (might be due to invalid IL or missing references)
		//IL_0057: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		ManeuverNode val = base.Vessel.patchedConicSolver.maneuverNodes[index];
		ManeuverNode val2 = base.Vessel.patchedConicSolver.maneuverNodes[index + 1];
		double ut = (val.UT + val2.UT) / 2.0;
		val.UpdateNode(val.patch.DeltaVToManeuverNodeCoordinates(ut, val.WorldDeltaV() + val2.WorldDeltaV()), ut);
		val2.RemoveSelf();
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_02ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_01de: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e8: Expected O, but got Unknown
		//IL_01e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f2: Expected O, but got Unknown
		//IL_0352: Unknown result type (might be due to invalid IL or missing references)
		//IL_0224: Unknown result type (might be due to invalid IL or missing references)
		//IL_022e: Expected O, but got Unknown
		//IL_022e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0238: Expected O, but got Unknown
		//IL_0406: Unknown result type (might be due to invalid IL or missing references)
		//IL_049e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0552: Unknown result type (might be due to invalid IL or missing references)
		//IL_05ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_07d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_083b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0887: Unknown result type (might be due to invalid IL or missing references)
		//IL_08fc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0943: Unknown result type (might be due to invalid IL or missing references)
		//IL_0bb8: Unknown result type (might be due to invalid IL or missing references)
		if (base.Vessel.patchedConicSolver.maneuverNodes.Count == 0)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_NodeEd_Label1"), Array.Empty<GUILayoutOption>());
			RelativityModeSelectUI();
			base.WindowGUI(windowID);
			return;
		}
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		ManeuverNode val = node;
		if (base.Vessel.patchedConicSolver.maneuverNodes.Count == 1)
		{
			node = base.Vessel.patchedConicSolver.maneuverNodes[0];
		}
		else
		{
			if (!base.Vessel.patchedConicSolver.maneuverNodes.Contains(node))
			{
				node = base.Vessel.patchedConicSolver.maneuverNodes[0];
			}
			int num = base.Vessel.patchedConicSolver.maneuverNodes.IndexOf(node);
			int count = base.Vessel.patchedConicSolver.maneuverNodes.Count;
			num = GuiUtils.ArrowSelector(num, count, "Maneuver node #" + (num + 1));
			node = base.Vessel.patchedConicSolver.maneuverNodes[num];
			if (num < count - 1 && GUILayout.Button(Localizer.Format("#MechJeb_NodeEd_button1"), Array.Empty<GUILayoutOption>()))
			{
				MergeNext(num);
			}
		}
		if (node != val)
		{
			prograde = node.DeltaV.z;
			radialPlus = node.DeltaV.x;
			normalPlus = node.DeltaV.y;
		}
		if ((Object)(object)gizmo != (Object)(object)node.attachedGizmo)
		{
			if ((Object)(object)gizmo != (Object)null)
			{
				ManeuverGizmo obj = gizmo;
				obj.OnGizmoUpdated = (HandlesUpdatedCallback)Delegate.Remove((Delegate)(object)obj.OnGizmoUpdated, (Delegate)new HandlesUpdatedCallback(GizmoUpdateHandler));
			}
			gizmo = node.attachedGizmo;
			if ((Object)(object)gizmo != (Object)null)
			{
				ManeuverGizmo obj2 = gizmo;
				obj2.OnGizmoUpdated = (HandlesUpdatedCallback)Delegate.Combine((Delegate)(object)obj2.OnGizmoUpdated, (Delegate)new HandlesUpdatedCallback(GizmoUpdateHandler));
			}
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_NodeEd_Label2"), prograde, "m/s", 60f);
		if (LimitedRepeatButtoon("-"))
		{
			prograde = (double)prograde - (double)progradeDelta;
			node.UpdateNode(new Vector3d((double)radialPlus, (double)normalPlus, (double)prograde), node.UT);
		}
		progradeDelta.Text = GUILayout.TextField(progradeDelta.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(50f) });
		if (LimitedRepeatButtoon("+"))
		{
			prograde = (double)prograde + (double)progradeDelta;
			node.UpdateNode(new Vector3d((double)radialPlus, (double)normalPlus, (double)prograde), node.UT);
		}
		GUILayout.Label("m/s", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_NodeEd_Label3"), radialPlus, "m/s", 60f);
		if (LimitedRepeatButtoon("-"))
		{
			radialPlus = (double)radialPlus - (double)radialPlusDelta;
			node.UpdateNode(new Vector3d((double)radialPlus, (double)normalPlus, (double)prograde), node.UT);
		}
		radialPlusDelta.Text = GUILayout.TextField(radialPlusDelta.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(50f) });
		if (LimitedRepeatButtoon("+"))
		{
			radialPlus = (double)radialPlus + (double)radialPlusDelta;
			node.UpdateNode(new Vector3d((double)radialPlus, (double)normalPlus, (double)prograde), node.UT);
		}
		GUILayout.Label("m/s", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_NodeEd_Label4"), normalPlus, "m/s", 60f);
		if (LimitedRepeatButtoon("-"))
		{
			normalPlus = (double)normalPlus - (double)normalPlusDelta;
			node.UpdateNode(new Vector3d((double)radialPlus, (double)normalPlus, (double)prograde), node.UT);
		}
		normalPlusDelta.Text = GUILayout.TextField(normalPlusDelta.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(50f) });
		if (LimitedRepeatButtoon("+"))
		{
			normalPlus = (double)normalPlus + (double)normalPlusDelta;
			node.UpdateNode(new Vector3d((double)radialPlus, (double)normalPlus, (double)prograde), node.UT);
		}
		GUILayout.Label("m/s", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_NodeEd_Label5"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		if (GUILayout.Button("0.01", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
		{
			progradeDelta = (radialPlusDelta = (normalPlusDelta = 0.01));
		}
		if (GUILayout.Button("0.1", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
		{
			progradeDelta = (radialPlusDelta = (normalPlusDelta = 0.1));
		}
		if (GUILayout.Button("1", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
		{
			progradeDelta = (radialPlusDelta = (normalPlusDelta = 1.0));
		}
		if (GUILayout.Button("10", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
		{
			progradeDelta = (radialPlusDelta = (normalPlusDelta = 10.0));
		}
		if (GUILayout.Button("100", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
		{
			progradeDelta = (radialPlusDelta = (normalPlusDelta = 100.0));
		}
		GUILayout.EndHorizontal();
		if (GUILayout.Button(Localizer.Format("#MechJeb_NodeEd_button2"), Array.Empty<GUILayoutOption>()))
		{
			node.UpdateNode(new Vector3d((double)radialPlus, (double)normalPlus, (double)prograde), node.UT);
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_NodeEd_Label6"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		if (GUILayout.Button("-o", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			node.UpdateNode(node.DeltaV, node.UT - node.patch.period);
		}
		if (GUILayout.Button("-", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			node.UpdateNode(node.DeltaV, node.UT - (double)timeOffset);
		}
		timeOffset.Text = GUILayout.TextField(timeOffset.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(100f) });
		if (GUILayout.Button("+", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			node.UpdateNode(node.DeltaV, node.UT + (double)timeOffset);
		}
		if (GUILayout.Button("+o", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
		{
			node.UpdateNode(node.DeltaV, node.UT + node.patch.period);
		}
		GUILayout.EndHorizontal();
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		if (GUILayout.Button(Localizer.Format("#MechJeb_NodeEd_button3"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth }))
		{
			Orbit patch = node.patch;
			double num2 = node.UT;
			switch (snap)
			{
			case Snap.PERIAPSIS:
				num2 = patch.NextPeriapsisTime((patch.eccentricity < 1.0) ? (num2 - patch.period / 2.0) : num2);
				break;
			case Snap.APOAPSIS:
				if (patch.eccentricity < 1.0)
				{
					num2 = patch.NextApoapsisTime(num2 - patch.period / 2.0);
				}
				break;
			case Snap.EQ_ASCENDING:
				if (patch.AscendingNodeEquatorialExists())
				{
					num2 = patch.TimeOfAscendingNodeEquatorial(num2 - patch.period / 2.0);
				}
				break;
			case Snap.EQ_DESCENDING:
				if (patch.DescendingNodeEquatorialExists())
				{
					num2 = patch.TimeOfDescendingNodeEquatorial(num2 - patch.period / 2.0);
				}
				break;
			case Snap.REL_ASCENDING:
				if (Core.Target.NormalTargetExists && (Object)(object)Core.Target.TargetOrbit.referenceBody == (Object)(object)patch.referenceBody && patch.AscendingNodeExists(Core.Target.TargetOrbit))
				{
					num2 = patch.TimeOfAscendingNode(Core.Target.TargetOrbit, num2 - patch.period / 2.0);
				}
				break;
			case Snap.REL_DESCENDING:
				if (Core.Target.NormalTargetExists && (Object)(object)Core.Target.TargetOrbit.referenceBody == (Object)(object)patch.referenceBody && patch.DescendingNodeExists(Core.Target.TargetOrbit))
				{
					num2 = patch.TimeOfDescendingNode(Core.Target.TargetOrbit, num2 - patch.period / 2.0);
				}
				break;
			}
			node.UpdateNode(node.DeltaV, num2);
		}
		snap = (Snap)GuiUtils.ArrowSelector((int)snap, numSnaps, snapStrings[(int)snap]);
		GUILayout.EndHorizontal();
		RelativityModeSelectUI();
		if (Core.Node != null)
		{
			if (base.Vessel.patchedConicSolver.maneuverNodes.Count > 0 && !Core.Node.Enabled)
			{
				if (GUILayout.Button(Localizer.Format("#MechJeb_NodeEd_button4"), Array.Empty<GUILayoutOption>()))
				{
					Core.Node.ExecuteOneNode(this);
				}
				if (base.Vessel.patchedConicSolver.maneuverNodes.Count > 1 && GUILayout.Button(Localizer.Format("#MechJeb_NodeEd_button5"), Array.Empty<GUILayoutOption>()))
				{
					Core.Node.ExecuteAllNodes(this);
				}
			}
			else if (Core.Node.Enabled && GUILayout.Button(Localizer.Format("#MechJeb_NodeEd_button6"), Array.Empty<GUILayoutOption>()))
			{
				Core.Node.Abort();
			}
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			Core.Node.Autowarp = GUILayout.Toggle(Core.Node.Autowarp, Localizer.Format("#MechJeb_NodeEd_checkbox1"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.EndHorizontal();
		}
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	private static bool LimitedRepeatButtoon(string text)
	{
		if (GUILayout.RepeatButton(text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }) && nextClick < Time.time)
		{
			nextClick = Time.time + 0.2f;
			return true;
		}
		return false;
	}

	private void RelativityModeSelectUI()
	{
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0051: Expected I4, but got Unknown
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_NodeEd_Label8"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		int num = GUILayout.SelectionGrid((int)base.Vessel.patchedConicRenderer.relativityMode, relativityModeStrings, 5, Array.Empty<GUILayoutOption>());
		base.Vessel.patchedConicRenderer.relativityMode = (RelativityMode)num;
		GUILayout.EndHorizontal();
		GUILayout.Label(Localizer.Format("#MechJeb_NodeEd_Label9", new string[1] { ((object)(RelativityMode)(ref base.Vessel.patchedConicRenderer.relativityMode)).ToString() }), Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
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
		return Localizer.Format("#MechJeb_NodeEd_title");
	}

	public override string IconName()
	{
		return "Maneuver Node Editor";
	}

	protected override bool IsSpaceCenterUpgradeUnlocked()
	{
		return base.Vessel.patchedConicsUnlocked();
	}

	public MechJebModuleNodeEditor(MechJebCore core)
		: base(core)
	{
	}
}
