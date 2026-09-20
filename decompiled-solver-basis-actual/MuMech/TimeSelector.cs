using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class TimeSelector
{
	private readonly string[] _timeRefNames;

	private double _universalTime;

	private readonly TimeReference[] _allowedTimeRef;

	[Persistent(pass = 4)]
	public int _currentTimeRef;

	[Persistent(pass = 4)]
	public readonly EditableTime LeadTime = 0.0;

	[Persistent(pass = 4)]
	public readonly EditableDoubleMult CircularizeAltitude = new EditableDoubleMult(150000.0, 1000.0);

	private static readonly string _maneuverException1 = Localizer.Format("#MechJeb_Maneu_Exception1");

	private static readonly string _maneuverException2 = Localizer.Format("#MechJeb_Maneu_Exception2");

	private static readonly string _maneuverException3 = Localizer.Format("#MechJeb_Maneu_Exception3");

	private static readonly string _maneuverException4 = Localizer.Format("#MechJeb_Maneu_Exception4");

	private static readonly string _maneuverException5 = Localizer.Format("#MechJeb_Maneu_Exception5");

	private static readonly string _maneuverException6 = Localizer.Format("#MechJeb_Maneu_Exception6");

	private static readonly string _maneuverException7 = Localizer.Format("#MechJeb_Maneu_Exception7");

	public TimeReference TimeReference => _allowedTimeRef[_currentTimeRef];

	public TimeSelector(TimeReference[] allowedTimeRef)
	{
		_allowedTimeRef = allowedTimeRef;
		_universalTime = 0.0;
		_timeRefNames = new string[allowedTimeRef.Length];
		for (int i = 0; i < allowedTimeRef.Length; i++)
		{
			string[] timeRefNames = _timeRefNames;
			int num = i;
			timeRefNames[num] = allowedTimeRef[i] switch
			{
				TimeReference.COMPUTED => Localizer.Format("#MechJeb_Maneu_TimeSelect1"), 
				TimeReference.APOAPSIS => Localizer.Format("#MechJeb_Maneu_TimeSelect2"), 
				TimeReference.CLOSEST_APPROACH => Localizer.Format("#MechJeb_Maneu_TimeSelect3"), 
				TimeReference.EQ_ASCENDING => Localizer.Format("#MechJeb_Maneu_TimeSelect4"), 
				TimeReference.EQ_DESCENDING => Localizer.Format("#MechJeb_Maneu_TimeSelect5"), 
				TimeReference.PERIAPSIS => Localizer.Format("#MechJeb_Maneu_TimeSelect6"), 
				TimeReference.REL_ASCENDING => Localizer.Format("#MechJeb_Maneu_TimeSelect7"), 
				TimeReference.REL_DESCENDING => Localizer.Format("#MechJeb_Maneu_TimeSelect8"), 
				TimeReference.X_FROM_NOW => Localizer.Format("#MechJeb_Maneu_TimeSelect9"), 
				TimeReference.ALTITUDE => Localizer.Format("#MechJeb_Maneu_TimeSelect10"), 
				TimeReference.EQ_NEAREST_AD => Localizer.Format("#MechJeb_Maneu_TimeSelect11"), 
				TimeReference.EQ_HIGHEST_AD => Localizer.Format("#MechJeb_Maneu_TimeSelect12"), 
				TimeReference.REL_NEAREST_AD => Localizer.Format("#MechJeb_Maneu_TimeSelect13"), 
				TimeReference.REL_HIGHEST_AD => Localizer.Format("#MechJeb_Maneu_TimeSelect14"), 
				_ => _timeRefNames[i], 
			};
		}
	}

	public void DoChooseTimeGUI()
	{
		GUILayout.Label(Localizer.Format("#MechJeb_Maneu_STB"), Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		_currentTimeRef = GuiUtils.ComboBox.Box(_currentTimeRef, _timeRefNames, this);
		switch (TimeReference)
		{
		case TimeReference.X_FROM_NOW:
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_of"), LeadTime);
			break;
		case TimeReference.ALTITUDE:
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_of"), CircularizeAltitude, "km");
			break;
		}
		GUILayout.EndHorizontal();
	}

	public double ComputeManeuverTime(Orbit o, double ut, MechJebModuleTargetController target)
	{
		//IL_01e6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fa: Unknown result type (might be due to invalid IL or missing references)
		switch (_allowedTimeRef[_currentTimeRef])
		{
		case TimeReference.X_FROM_NOW:
			ut += LeadTime.Val;
			break;
		case TimeReference.APOAPSIS:
			if (!(o.eccentricity < 1.0))
			{
				throw new OperationException(_maneuverException1);
			}
			ut = o.NextApoapsisTime(ut);
			break;
		case TimeReference.PERIAPSIS:
			ut = o.NextPeriapsisTime(ut);
			break;
		case TimeReference.CLOSEST_APPROACH:
			if (!target.NormalTargetExists)
			{
				throw new OperationException(_maneuverException2);
			}
			ut = o.NextClosestApproachTime(target.TargetOrbit, ut);
			break;
		case TimeReference.ALTITUDE:
			if (!((double)CircularizeAltitude > o.PeA) || (!((double)CircularizeAltitude < o.ApA) && !(o.eccentricity >= 1.0)))
			{
				throw new OperationException(_maneuverException3);
			}
			ut = o.NextTimeOfRadius(ut, o.referenceBody.Radius + (double)CircularizeAltitude);
			break;
		case TimeReference.EQ_ASCENDING:
			if (!o.AscendingNodeEquatorialExists())
			{
				throw new OperationException(_maneuverException4);
			}
			ut = o.TimeOfAscendingNodeEquatorial(ut);
			break;
		case TimeReference.EQ_DESCENDING:
			if (!o.DescendingNodeEquatorialExists())
			{
				throw new OperationException(_maneuverException5);
			}
			ut = o.TimeOfDescendingNodeEquatorial(ut);
			break;
		case TimeReference.EQ_NEAREST_AD:
			if (o.AscendingNodeEquatorialExists() && o.DescendingNodeEquatorialExists())
			{
				ut = Math.Min(o.TimeOfAscendingNodeEquatorial(ut), o.TimeOfDescendingNodeEquatorial(ut));
				break;
			}
			if (o.AscendingNodeEquatorialExists())
			{
				ut = o.TimeOfAscendingNodeEquatorial(ut);
				break;
			}
			if (o.DescendingNodeEquatorialExists())
			{
				ut = o.TimeOfDescendingNodeEquatorial(ut);
				break;
			}
			throw new OperationException(_maneuverException6);
		case TimeReference.EQ_HIGHEST_AD:
			if (o.AscendingNodeEquatorialExists() && o.DescendingNodeEquatorialExists())
			{
				double num = o.TimeOfAscendingNodeEquatorial(ut);
				double num2 = o.TimeOfDescendingNodeEquatorial(ut);
				Vector3d orbitalVelocityAtUT = o.getOrbitalVelocityAtUT(num);
				double magnitude = ((Vector3d)(ref orbitalVelocityAtUT)).magnitude;
				orbitalVelocityAtUT = o.getOrbitalVelocityAtUT(num2);
				ut = ((magnitude <= ((Vector3d)(ref orbitalVelocityAtUT)).magnitude) ? num : num2);
			}
			else if (o.AscendingNodeEquatorialExists())
			{
				ut = o.TimeOfAscendingNodeEquatorial(ut);
			}
			else
			{
				if (!o.DescendingNodeEquatorialExists())
				{
					throw new OperationException(_maneuverException7);
				}
				ut = o.TimeOfDescendingNodeEquatorial(ut);
			}
			break;
		}
		_universalTime = ut;
		return _universalTime;
	}
}
