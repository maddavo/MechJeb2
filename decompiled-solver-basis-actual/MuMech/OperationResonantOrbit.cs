using System;
using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class OperationResonantOrbit : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_resonant_title");

	[Persistent(pass = 4)]
	public EditableInt ResonanceNumerator = 2;

	[Persistent(pass = 4)]
	public EditableInt ResonanceDenominator = 3;

	private readonly TimeSelector _timeSelector = new TimeSelector(new TimeReference[4]
	{
		TimeReference.APOAPSIS,
		TimeReference.PERIAPSIS,
		TimeReference.X_FROM_NOW,
		TimeReference.ALTITUDE
	});

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		GUILayout.Label(Localizer.Format("#MechJeb_resonant_label1", new string[1] { ResonanceNumerator.Val + "/" + ResonanceDenominator.Val }), Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_resonant_label2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		ResonanceNumerator.Text = GUILayout.TextField(ResonanceNumerator.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(30f) });
		GUILayout.Label("/", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		ResonanceDenominator.Text = GUILayout.TextField(ResonanceDenominator.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(30f) });
		GUILayout.EndHorizontal();
		_timeSelector.DoChooseTimeGUI();
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0036: Unknown result type (might be due to invalid IL or missing references)
		double num = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		Vector3d dV = OrbitalManeuverCalculator.DeltaVToResonantOrbit(o, num, (double)ResonanceNumerator.Val / (double)ResonanceDenominator.Val);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, num)
		};
	}
}
