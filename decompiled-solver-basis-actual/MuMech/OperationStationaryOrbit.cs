using System;
using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class OperationStationaryOrbit : Operation
{
	[Persistent]
	public double targetLongitude;

	[Persistent]
	public double targetLatitude;

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_stationary_title");
	}

	private void MoveByMeter(ref EditableAngle angle, double distance, double alt, Orbit o)
	{
		double num = distance * (180.0 / Math.PI) / (alt + o.referenceBody.Radius);
		angle = (double)angle + num;
	}

	public override void DoParametersGUI(Orbit o, double UT, MechJebModuleTargetController targetController)
	{
		double alt = o.referenceBody.TerrainAltitude((double)targetController.targetLatitude, (double)targetController.targetLongitude, false);
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		if (!targetController.PositionTargetExists)
		{
			targetController.SetPositionTarget(o.referenceBody, targetLatitude, targetLongitude);
		}
		GUILayout.Label(Localizer.Format("#MechJeb_stationary_label1"), Array.Empty<GUILayoutOption>());
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		targetController.targetLongitude.DrawEditGUI(EditableAngle.Direction.EW);
		if (GUILayout.Button("◄", Array.Empty<GUILayoutOption>()))
		{
			MoveByMeter(ref targetController.targetLongitude, -10.0, alt, o);
		}
		GUILayout.Label("10m", Array.Empty<GUILayoutOption>());
		if (GUILayout.Button("►", Array.Empty<GUILayoutOption>()))
		{
			MoveByMeter(ref targetController.targetLongitude, 10.0, alt, o);
		}
		GUILayout.EndHorizontal();
		if ((Object)(object)targetController.targetBody != (Object)null)
		{
			GUILayout.Label(targetController.targetBody.GetExperimentBiomeSafe(targetController.targetLatitude, targetController.targetLongitude), Array.Empty<GUILayoutOption>());
		}
		if (GUILayout.Button(Localizer.Format("#MechJeb_LandingGuidance_button2"), Array.Empty<GUILayoutOption>()))
		{
			targetController.PickPositionTargetOnMap();
		}
		targetLatitude = 0.0;
		targetLongitude = targetController.targetLongitude;
		GUILayout.EndVertical();
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double ut, MechJebModuleTargetController target)
	{
		//IL_010c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0117: Unknown result type (might be due to invalid IL or missing references)
		//IL_011c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0121: Unknown result type (might be due to invalid IL or missing references)
		//IL_013c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0141: Unknown result type (might be due to invalid IL or missing references)
		//IL_0142: Unknown result type (might be due to invalid IL or missing references)
		//IL_0147: Unknown result type (might be due to invalid IL or missing references)
		//IL_014b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0152: Unknown result type (might be due to invalid IL or missing references)
		//IL_0157: Unknown result type (might be due to invalid IL or missing references)
		//IL_0159: Unknown result type (might be due to invalid IL or missing references)
		//IL_0160: Expected O, but got Unknown
		//IL_0162: Unknown result type (might be due to invalid IL or missing references)
		//IL_0163: Unknown result type (might be due to invalid IL or missing references)
		//IL_01af: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bf: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d3: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e2: Unknown result type (might be due to invalid IL or missing references)
		double num = Math.Pow(o.referenceBody.gravParameter * Math.Pow(o.referenceBody.rotationPeriod / (Math.PI * 2.0), 2.0), 1.0 / 3.0);
		if (num > o.referenceBody.sphereOfInfluence)
		{
			throw new OperationException(Localizer.Format("#MechJeb_stationary_Exception1", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.displayName) }));
		}
		double num2 = (o.referenceBody.rotationAngle + 360.0 * (ut / o.referenceBody.rotationPeriod)) * Math.PI / 180.0;
		_ = ((targetLongitude * Math.PI / 180.0 + num2) % (Math.PI * 2.0) + Math.PI * 2.0) % (Math.PI * 2.0);
		double num3 = num - o.referenceBody.Radius;
		Vector3d val = o.referenceBody.GetWorldSurfacePosition(0.0, targetLongitude, num3) - o.referenceBody.position;
		double num4 = Math.Sqrt(o.referenceBody.gravParameter / num);
		Vector3d val2 = Vector3d.Cross(o.referenceBody.angularVelocity, val);
		Vector3d val3 = ((Vector3d)(ref val2)).normalized * num4;
		Orbit val4 = new Orbit();
		val4.UpdateFromStateVectors(val, val3, o.referenceBody, ut);
		val4.eccentricity = 0.0;
		val4.inclination = 0.0;
		val4.Init();
		var (dV, ut2, dV2, ut3) = OrbitalManeuverCalculator.DeltaVAndTimeForHohmannTransfer(o, val4, ut, 0.0, fixedTime: false, coplanar: false);
		return new List<ManeuverParameters>
		{
			new ManeuverParameters(dV, ut2),
			new ManeuverParameters(dV2, ut3)
		};
	}
}
