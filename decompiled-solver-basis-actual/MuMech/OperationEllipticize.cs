using System.Collections.Generic;
using KSP.Localization;
using MechJebLib.Utils;

namespace MuMech;

public class OperationEllipticize : Operation
{
	private static readonly string _name = Localizer.Format("#MechJeb_both_title");

	[Persistent(pass = 4)]
	public EditableDoubleMult NewApA = new EditableDoubleMult(200000.0, 1000.0);

	[Persistent(pass = 4)]
	public EditableDoubleMult NewPeA = new EditableDoubleMult(100000.0, 1000.0);

	private static readonly TimeReference[] _timeReferences = new TimeReference[3]
	{
		TimeReference.APOAPSIS,
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
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_both_label1"), NewPeA, "km");
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_both_label2"), NewApA, "km");
		_timeSelector.DoChooseTimeGUI();
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_014c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0151: Unknown result type (might be due to invalid IL or missing references)
		//IL_0159: Unknown result type (might be due to invalid IL or missing references)
		double ut = _timeSelector.ComputeManeuverTime(o, universalTime, target);
		string text = Statics.ToSI(o.Radius(ut) - o.referenceBody.Radius, 4, int.MaxValue) + "m";
		if (o.referenceBody.Radius + (double)NewPeA > o.Radius(ut))
		{
			throw new OperationException(Localizer.Format("#MechJeb_both_Exception1", new string[1] { text }));
		}
		if (o.referenceBody.Radius + (double)NewApA < o.Radius(ut))
		{
			throw new OperationException(Localizer.Format("#MechJeb_both_Exception2") + "(" + text + ")");
		}
		if ((double)NewPeA < 0.0 - o.referenceBody.Radius)
		{
			throw new OperationException(Localizer.Format("#MechJeb_both_Exception3", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.displayName) }) + "(-" + Statics.ToSI(o.referenceBody.Radius, 3, int.MaxValue) + "m)");
		}
		double newPeR = (double)NewPeA + o.referenceBody.Radius;
		double newApR = (double)NewApA + o.referenceBody.Radius;
		Vector3d dV = OrbitalManeuverCalculator.DeltaVToEllipticize(o, ut, newPeR, newApR);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, ut)
		};
	}
}
