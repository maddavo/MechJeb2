using System;
using System.Collections.Generic;
using System.Linq;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class MechJebModuleLandingGuidance : DisplayModule
{
	public struct LandingSite
	{
		public string Name;

		public CelestialBody Body;

		public double Latitude;

		public double Longitude;
	}

	private MechJebModuleLandingPredictions _predictor;

	public static List<LandingSite> LandingSites;

	[Persistent(pass = 5)]
	public int _landingSiteIdx;

	public override void OnStart(StartState state)
	{
		_predictor = Core.GetComputerModule<MechJebModuleLandingPredictions>();
		if (LandingSites == null && HighLogic.LoadedSceneIsFlight)
		{
			InitLandingSitesList();
		}
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(200f),
			GUILayout.Height(150f)
		};
	}

	private void MoveByMeter(ref EditableAngle angle, double distance, double alt)
	{
		double num = distance * (180.0 / Math.PI) / (alt + base.MainBody.Radius);
		angle = (double)angle + num;
	}

	protected override void WindowGUI(int windowID)
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		if (Core.Target.PositionTargetExists)
		{
			double num = ((PartModule)Core).vessel.mainBody.TerrainAltitude((double)Core.Target.targetLatitude, (double)Core.Target.targetLongitude, false);
			GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_label1"), Array.Empty<GUILayoutOption>());
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			Core.Target.targetLatitude.DrawEditGUI(EditableAngle.Direction.NS);
			if (GUILayout.Button("▲", Array.Empty<GUILayoutOption>()))
			{
				MoveByMeter(ref Core.Target.targetLatitude, 10.0, num);
			}
			GUILayout.Label("10m", Array.Empty<GUILayoutOption>());
			if (GUILayout.Button("▼", Array.Empty<GUILayoutOption>()))
			{
				MoveByMeter(ref Core.Target.targetLatitude, -10.0, num);
			}
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			Core.Target.targetLongitude.DrawEditGUI(EditableAngle.Direction.EW);
			if (GUILayout.Button("◄", Array.Empty<GUILayoutOption>()))
			{
				MoveByMeter(ref Core.Target.targetLongitude, -10.0, num);
			}
			GUILayout.Label("10m", Array.Empty<GUILayoutOption>());
			if (GUILayout.Button("►", Array.Empty<GUILayoutOption>()))
			{
				MoveByMeter(ref Core.Target.targetLongitude, 10.0, num);
			}
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label("ASL: " + Statics.ToSI(num, 4, int.MaxValue) + "m", Array.Empty<GUILayoutOption>());
			GUILayout.Label(Core.Target.targetBody.GetExperimentBiomeSafe(Core.Target.targetLatitude, Core.Target.targetLongitude), Array.Empty<GUILayoutOption>());
			GUILayout.EndHorizontal();
		}
		else if (GUILayout.Button(Localizer.Format("#MechJeb_LandingGuidance_button1"), Array.Empty<GUILayoutOption>()))
		{
			Core.Target.SetPositionTarget(base.MainBody, Core.Target.targetLatitude, Core.Target.targetLongitude);
		}
		if (GUILayout.Button(Localizer.Format("#MechJeb_LandingGuidance_button2"), Array.Empty<GUILayoutOption>()))
		{
			Core.Target.PickPositionTargetOnMap();
		}
		List<LandingSite> list = LandingSites.Where((LandingSite p) => (Object)(object)p.Body == (Object)(object)base.MainBody).ToList();
		if (list.Any())
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			_landingSiteIdx = GuiUtils.ComboBox.Box(_landingSiteIdx, list.Select((LandingSite p) => p.Name).ToArray(), this);
			if (GUILayout.Button("Set", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth }))
			{
				Core.Target.SetPositionTarget(base.MainBody, list[_landingSiteIdx].Latitude, list[_landingSiteIdx].Longitude);
			}
			GUILayout.EndHorizontal();
		}
		DrawGUITogglePredictions();
		if (Core.Landing != null)
		{
			GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_label2"), Array.Empty<GUILayoutOption>());
			_predictor.maxOrbits = (Core.Landing.Enabled ? 0.5 : 4.0);
			_predictor.noSkipToFreefall = !Core.Landing.Enabled;
			if (Core.Landing.Enabled)
			{
				if (GUILayout.Button(Localizer.Format("#MechJeb_LandingGuidance_button3"), Array.Empty<GUILayoutOption>()))
				{
					Core.Landing.StopLanding();
				}
			}
			else
			{
				GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
				if (!Core.Target.PositionTargetExists || base.Vessel.LandedOrSplashed)
				{
					GUI.enabled = false;
				}
				if (GUILayout.Button(Localizer.Format("#MechJeb_LandingGuidance_button4"), Array.Empty<GUILayoutOption>()))
				{
					Core.Landing.LandAtPositionTarget(this);
				}
				GUI.enabled = !base.Vessel.LandedOrSplashed;
				if (GUILayout.Button(Localizer.Format("#MechJeb_LandingGuidance_button5"), Array.Empty<GUILayoutOption>()))
				{
					Core.Landing.LandUntargeted(this);
				}
				GUI.enabled = true;
				GUILayout.EndHorizontal();
			}
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_LandingGuidance_label3"), Core.Landing.TouchdownSpeed, "m/s", 35f);
			if (Core.Landing != null)
			{
				Core.Node.Autowarp = GUILayout.Toggle(Core.Node.Autowarp, Localizer.Format("#MechJeb_LandingGuidance_checkbox1"), Array.Empty<GUILayoutOption>());
			}
			Core.Landing.DeployGears = GUILayout.Toggle(Core.Landing.DeployGears, Localizer.Format("#MechJeb_LandingGuidance_checkbox2"), Array.Empty<GUILayoutOption>());
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_LandingGuidance_label4"), Core.Landing.LimitGearsStage, "", 35f);
			Core.Landing.DeployChutes = GUILayout.Toggle(Core.Landing.DeployChutes, Localizer.Format("#MechJeb_LandingGuidance_checkbox3"), Array.Empty<GUILayoutOption>());
			_predictor.deployChutes = Core.Landing.DeployChutes;
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_LandingGuidance_label5"), Core.Landing.LimitChutesStage, "", 35f);
			_predictor.limitChutesStage = Core.Landing.LimitChutesStage;
			Core.Landing.RCSAdjustment = GUILayout.Toggle(Core.Landing.RCSAdjustment, Localizer.Format("#MechJeb_LandingGuidance_checkbox4"), Array.Empty<GUILayoutOption>());
			Core.Thrust.LimiterMinThrottleInfoItem();
			if (Core.Landing.Enabled)
			{
				GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_label6") + Core.Landing.Status, Array.Empty<GUILayoutOption>());
				GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_label7") + ((Core.Landing.CurrentStep != null) ? Core.Landing.CurrentStep.GetType().Name : "N/A"), Array.Empty<GUILayoutOption>());
				GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_label8") + ((Core.Landing.DescentSpeedPolicy != null) ? Core.Landing.DescentSpeedPolicy.GetType().Name : "N/A") + " (" + Core.Landing.UseAtmosphereToBrake() + ")", Array.Empty<GUILayoutOption>());
			}
		}
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	public void SetAndLandTargetKSC()
	{
		LandingSite landingSite = LandingSites.First((LandingSite x) => x.Name == "KSC Pad");
		Core.Target.SetPositionTarget(base.MainBody, landingSite.Latitude, landingSite.Longitude);
		Core.Landing.LandAtPositionTarget(this);
	}

	public void LandSomewhere()
	{
		Core.Landing.StopLanding();
		Core.Landing.LandUntargeted(this);
	}

	[GeneralInfoItem("#MechJeb_LandingPredictions", InfoItem.Category.Misc)]
	private void DrawGUITogglePredictions()
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		bool flag = GUILayout.Toggle(_predictor.Enabled, Localizer.Format("#MechJeb_LandingGuidance_checkbox5"), Array.Empty<GUILayoutOption>());
		if (_predictor.Enabled != flag)
		{
			if (flag)
			{
				_predictor.Users.Add(this);
			}
			else
			{
				_predictor.Users.Remove(this);
			}
		}
		if (_predictor.Enabled)
		{
			_predictor.makeAerobrakeNodes = GUILayout.Toggle(_predictor.makeAerobrakeNodes, Localizer.Format("#MechJeb_LandingGuidance_checkbox6"), Array.Empty<GUILayoutOption>());
			_predictor.showTrajectory = GUILayout.Toggle(_predictor.showTrajectory, Localizer.Format("#MechJeb_LandingGuidance_checkbox7"), Array.Empty<GUILayoutOption>());
			_predictor.worldTrajectory = GUILayout.Toggle(_predictor.worldTrajectory, Localizer.Format("#MechJeb_LandingGuidance_checkbox8"), Array.Empty<GUILayoutOption>());
			_predictor.camTrajectory = GUILayout.Toggle(_predictor.camTrajectory, Localizer.Format("#MechJeb_LandingGuidance_checkbox9"), Array.Empty<GUILayoutOption>());
			DrawGUIPrediction();
		}
		GUILayout.EndVertical();
	}

	private void DrawGUIPrediction()
	{
		//IL_00da: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_0128: Unknown result type (might be due to invalid IL or missing references)
		//IL_0133: Unknown result type (might be due to invalid IL or missing references)
		//IL_0138: Unknown result type (might be due to invalid IL or missing references)
		ReentrySimulation.Result result = _predictor.Result;
		if (result == null)
		{
			return;
		}
		switch (result.Outcome)
		{
		case ReentrySimulation.Outcome.LANDED:
		{
			GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_label9"), Array.Empty<GUILayoutOption>());
			GUILayout.Label(Coordinates.ToStringDMS(result.EndPosition.Latitude, result.EndPosition.Longitude) + "\nASL:" + Statics.ToSI(result.EndASL, 4, int.MaxValue) + "m", Array.Empty<GUILayoutOption>());
			GUILayout.Label(result.Body.GetExperimentBiomeSafe(result.EndPosition.Latitude, result.EndPosition.Longitude), Array.Empty<GUILayoutOption>());
			double num = Vector3d.Distance(base.MainBody.GetWorldSurfacePosition(result.EndPosition.Latitude, result.EndPosition.Longitude, 0.0) - base.MainBody.position, base.MainBody.GetWorldSurfacePosition((double)Core.Target.targetLatitude, (double)Core.Target.targetLongitude, 0.0) - base.MainBody.position);
			GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_Label10") + Statics.ToSI(num, 4, int.MaxValue) + "m" + Localizer.Format("#MechJeb_LandingGuidance_Label11") + result.MaxDragGees.ToString("F1") + "g" + Localizer.Format("#MechJeb_LandingGuidance_Label12") + result.DeltaVExpended.ToString("F1") + "m/s" + Localizer.Format("#MechJeb_LandingGuidance_Label13") + (base.Vessel.Landed ? "0.0s" : GuiUtils.TimeToDHMS(result.EndUT - Planetarium.GetUniversalTime(), 1)), Array.Empty<GUILayoutOption>());
			break;
		}
		case ReentrySimulation.Outcome.AEROBRAKED:
		{
			GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_Label14"), Array.Empty<GUILayoutOption>());
			Orbit val = result.AeroBrakeOrbit();
			if (val.eccentricity > 1.0)
			{
				GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_Label15") + val.eccentricity.ToString("F2"), Array.Empty<GUILayoutOption>());
			}
			else
			{
				GUILayout.Label(Statics.ToSI(val.PeA, 3, int.MaxValue) + "m x " + Statics.ToSI(val.ApA, 3, int.MaxValue) + "m", Array.Empty<GUILayoutOption>());
			}
			GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_Label16", new string[1] { result.MaxDragGees.ToString("F1") }) + Localizer.Format("#MechJeb_LandingGuidance_Label17", new string[1] { GuiUtils.TimeToDHMS(result.AeroBrakeUT - Planetarium.GetUniversalTime(), 1) }), Array.Empty<GUILayoutOption>());
			break;
		}
		case ReentrySimulation.Outcome.NO_REENTRY:
			GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_Label18_1") + Statics.ToSI(base.Orbit.PeA, 3, int.MaxValue) + "m Pe > " + Statics.ToSI(base.MainBody.RealMaxAtmosphereAltitude(), 3, int.MaxValue) + (base.MainBody.atmosphere ? Localizer.Format("#MechJeb_LandingGuidance_Label18_2") : Localizer.Format("#MechJeb_LandingGuidance_Label18_3")), Array.Empty<GUILayoutOption>());
			break;
		case ReentrySimulation.Outcome.TIMED_OUT:
			GUILayout.Label(Localizer.Format("#MechJeb_LandingGuidance_Label19"), Array.Empty<GUILayoutOption>());
			break;
		}
	}

	private void InitLandingSitesList()
	{
		//IL_02ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0304: Unknown result type (might be due to invalid IL or missing references)
		//IL_0308: Unknown result type (might be due to invalid IL or missing references)
		//IL_030d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0328: Unknown result type (might be due to invalid IL or missing references)
		//IL_0340: Unknown result type (might be due to invalid IL or missing references)
		//IL_0347: Unknown result type (might be due to invalid IL or missing references)
		LandingSites = new List<LandingSite>();
		UrlConfig[] configs = GameDatabase.Instance.GetConfigs("MechJeb2Landing");
		for (int i = 0; i < configs.Length; i++)
		{
			ConfigNode[] nodes = configs[i].config.GetNode("LandingSites").GetNodes("Site");
			foreach (ConfigNode val in nodes)
			{
				ComputerModule.Print("site " + (object)val);
				string launchSiteName3 = val.GetValue("name");
				string value = val.GetValue("latitude");
				string value2 = val.GetValue("longitude");
				if (launchSiteName3 == null || value == null || value2 == null)
				{
					ComputerModule.Print("Ignore landing site with null value");
					continue;
				}
				double.TryParse(value, out var result);
				double.TryParse(value2, out var result2);
				string bodyName2 = val.GetValue("body");
				CelestialBody body = ((bodyName2 != null) ? FlightGlobals.Bodies.Find((CelestialBody b) => b.bodyName == bodyName2) : Planetarium.fetch.Home);
				if (LandingSites.All((LandingSite p) => p.Name != launchSiteName3))
				{
					ComputerModule.Print("Adding " + launchSiteName3);
					LandingSites.Add(new LandingSite
					{
						Name = launchSiteName3,
						Latitude = result,
						Longitude = result2,
						Body = body
					});
				}
			}
		}
		foreach (LaunchSite launchSite in PSystemSetup.Instance.LaunchSites)
		{
			if (launchSite.spawnPoints.Length != 0)
			{
				SpawnPoint val2 = launchSite.spawnPoints[0];
				LandingSites.Add(new LandingSite
				{
					Name = val2.name.Replace("_", " "),
					Latitude = val2.latitude,
					Longitude = val2.longitude,
					Body = launchSite.Body
				});
			}
		}
		configs = GameDatabase.Instance.GetConfigs("STATIC");
		for (int i = 0; i < configs.Length; i++)
		{
			ConfigNode[] nodes = configs[i].config.GetNodes("Instances");
			foreach (ConfigNode val3 in nodes)
			{
				string bodyName = val3.GetValue("CelestialBody");
				string value3 = val3.GetValue("RadialPosition");
				string launchSiteName2 = val3.GetValue("LaunchSiteName");
				string value4 = val3.GetValue("LaunchSiteType");
				if (bodyName != null && value3 != null && launchSiteName2 != null && value4 != null && !(value4 != "VAB"))
				{
					Vector3d val4 = ConfigNode.ParseVector3D(value3);
					Vector3d normalized = ((Vector3d)(ref val4)).normalized;
					CelestialBody val5 = FlightGlobals.Bodies.Find((CelestialBody b) => b.bodyName == bodyName);
					double num = Math.Asin(normalized.y) * (180.0 / Math.PI);
					double num2 = Math.Atan2(normalized.z, normalized.x) * (180.0 / Math.PI);
					if ((Object)(object)val5 != (Object)null && LandingSites.All((LandingSite p) => p.Name != launchSiteName2))
					{
						LandingSites.Add(new LandingSite
						{
							Name = launchSiteName2,
							Latitude = ((!double.IsNaN(num)) ? num : 0.0),
							Longitude = ((!double.IsNaN(num2)) ? num2 : 0.0),
							Body = val5
						});
					}
				}
			}
		}
		UrlConfig val6 = GameDatabase.Instance.GetConfigs("KSCSWITCHER").FirstOrDefault();
		if (val6 != null)
		{
			ConfigNode node = val6.config.GetNode("LaunchSites");
			if (node != null)
			{
				ConfigNode[] nodes = node.GetNodes("Site");
				foreach (ConfigNode val7 in nodes)
				{
					string launchSiteName = val7.GetValue("displayName");
					ConfigNode node2 = val7.GetNode("PQSCity");
					if (node2 == null)
					{
						continue;
					}
					string value5 = node2.GetValue("latitude");
					string value6 = node2.GetValue("longitude");
					if (launchSiteName != null && value5 != null && value6 != null)
					{
						double.TryParse(value5, out var result3);
						double.TryParse(value6, out var result4);
						if (LandingSites.All((LandingSite p) => p.Name != launchSiteName))
						{
							LandingSites.Add(new LandingSite
							{
								Name = launchSiteName,
								Latitude = result3,
								Longitude = result4,
								Body = Planetarium.fetch.Home
							});
						}
					}
				}
			}
		}
		if (_landingSiteIdx > LandingSites.Count)
		{
			_landingSiteIdx = 0;
		}
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_LandingGuidance_title");
	}

	public override string IconName()
	{
		return "Landing Guidance";
	}

	protected override bool IsSpaceCenterUpgradeUnlocked()
	{
		return base.Vessel.patchedConicsUnlocked();
	}

	public MechJebModuleLandingGuidance(MechJebCore core)
		: base(core)
	{
	}
}
