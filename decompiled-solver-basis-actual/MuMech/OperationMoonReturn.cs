using System;
using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class OperationMoonReturn : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_return_title");

	[Persistent(pass = 4)]
	public EditableDoubleMult Periapsis = new EditableDoubleMult(100000.0, 1000.0);

	[Persistent(pass = 4)]
	public EditableDouble Inclination = new EditableDouble(-90.0);

	[Persistent(pass = 4)]
	public bool InclinationFlag;

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_return_label1"), Periapsis, "km");
		GuiUtils.ToggledTextBox(ref InclinationFlag, "Inclination", Inclination, "°");
		GUILayout.Label(Localizer.Format("#MechJeb_return_label2"), Array.Empty<GUILayoutOption>());
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_0082: Unknown result type (might be due to invalid IL or missing references)
		//IL_0087: Unknown result type (might be due to invalid IL or missing references)
		//IL_0094: Unknown result type (might be due to invalid IL or missing references)
		if ((Object)(object)o.referenceBody.referenceBody == (Object)null)
		{
			throw new OperationException(Localizer.Format("#MechJeb_return_Exception", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.displayName) }));
		}
		double peR = o.referenceBody.referenceBody.Radius + (double)Periapsis;
		double inc = (InclinationFlag ? Inclination.Val : double.NaN);
		var (dV, ut) = OrbitalManeuverCalculator.DeltaVAndTimeForMoonReturnEjection(o, universalTime, peR, inc);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, ut)
		};
	}
}
