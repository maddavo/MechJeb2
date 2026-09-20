using System.Collections.Generic;
using KSP.Localization;
using MechJebLib.Utils;

namespace MuMech;

public class OperationSemiMajor : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_Sa_title");

	[Persistent(pass = 4)]
	public EditableDoubleMult NewSma = new EditableDoubleMult(800000.0, 1000.0);

	private static readonly TimeReference[] _timeReferences = new TimeReference[4]
	{
		TimeReference.APOAPSIS,
		TimeReference.PERIAPSIS,
		TimeReference.X_FROM_NOW,
		TimeReference.ALTITUDE
	};

	private static readonly TimeSelector _timeSelector = new TimeSelector(_timeReferences);

	public override string GetName()
	{
		return _name;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_Sa_label"), NewSma, "km");
		_timeSelector.DoChooseTimeGUI();
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_00c7: Unknown result type (might be due to invalid IL or missing references)
		double ut = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		if (2.0 * (double)NewSma > o.Radius(ut) + o.referenceBody.sphereOfInfluence)
		{
			ErrorMessage = Localizer.Format("#MechJeb_Sa_errormsg");
		}
		if (o.Radius(ut) > 2.0 * (double)NewSma)
		{
			throw new OperationException(Localizer.Format("#MechJeb_Sa_Exception", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.displayName) }) + "(" + Statics.ToSI(o.referenceBody.Radius, 3, int.MaxValue) + "m)");
		}
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(OrbitalManeuverCalculator.DeltaVForSemiMajorAxis(o, ut, NewSma), ut)
		};
	}
}
