using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace MuMech;

public class MechJebModuleRoverController : ComputerModule
{
	public readonly List<MechJebWaypoint> Waypoints = new List<MechJebWaypoint>();

	public int WaypointIndex = -1;

	private CelestialBody lastBody;

	public bool LoopWaypoints;

	[ToggleInfoItem("#MechJeb_ControlHeading", InfoItem.Category.Rover)]
	[Persistent(pass = 1)]
	public bool ControlHeading;

	[EditableInfoItem("#MechJeb_Heading", InfoItem.Category.Rover, width = 40f)]
	[Persistent(pass = 1)]
	public EditableDouble heading = 0.0;

	[ToggleInfoItem("#MechJeb_ControlSpeed", InfoItem.Category.Rover)]
	[Persistent(pass = 1)]
	public bool ControlSpeed;

	[EditableInfoItem("#MechJeb_Speed", InfoItem.Category.Rover, width = 40f)]
	[Persistent(pass = 1)]
	public readonly EditableDouble speed = 10.0;

	[ToggleInfoItem("#MechJeb_BrakeOnEject", InfoItem.Category.Rover)]
	[Persistent(pass = 1)]
	public readonly bool BrakeOnEject;

	[ToggleInfoItem("#MechJeb_BrakeOnEnergyDepletion", InfoItem.Category.Rover)]
	[Persistent(pass = 1)]
	public readonly bool BrakeOnEnergyDepletion;

	[ToggleInfoItem("#MechJeb_WarpToDaylight", InfoItem.Category.Rover)]
	[Persistent(pass = 1)]
	public readonly bool WarpToDaylight;

	public bool waitingForDaylight;

	[ToggleInfoItem("#MechJeb_StabilityControl", InfoItem.Category.Rover)]
	[Persistent(pass = 1)]
	public readonly bool StabilityControl;

	[ToggleInfoItem("#MechJeb_LimitAcceleration", InfoItem.Category.Rover)]
	[Persistent(pass = 3)]
	public bool LimitAcceleration;

	public PIDController headingPID;

	public PIDController speedPID;

	[EditableInfoItem("#MechJeb_SafeTurnspeed", InfoItem.Category.Rover)]
	[Persistent(pass = 2)]
	public readonly EditableDouble turnSpeed = 3.0;

	[EditableInfoItem("#MechJeb_TerrainLookAhead", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble terrainLookAhead = 1.0;

	[EditableInfoItem("#MechJeb_BrakeSpeedLimit", InfoItem.Category.Rover)]
	[Persistent(pass = 2)]
	public readonly EditableDouble brakeSpeedLimit = 0.7;

	[EditableInfoItem("#MechJeb_HeadingPIDP", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble hPIDp = 0.03;

	[EditableInfoItem("#MechJeb_HeadingPIDI", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble hPIDi = 0.002;

	[EditableInfoItem("#MechJeb_HeadingPIDD", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble hPIDd = 0.005;

	[EditableInfoItem("#MechJeb_SpeedPIDP", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble sPIDp = 2.0;

	[EditableInfoItem("#MechJeb_SpeedPIDI", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble sPIDi = 0.1;

	[EditableInfoItem("#MechJeb_SpeedPIDD", InfoItem.Category.Rover)]
	[Persistent(pass = 4)]
	public readonly EditableDouble sPIDd = 0.001;

	[ValueInfoItem("#MechJeb_SpeedIntAcc", InfoItem.Category.Rover, format = "SI", units = "m/s")]
	public double speedIntAcc;

	[ValueInfoItem("#MechJeb_Traction", InfoItem.Category.Rover, format = "F0", units = "%")]
	public float traction;

	[EditableInfoItem("#MechJeb_TractionBrakeLimit", InfoItem.Category.Rover)]
	[Persistent(pass = 2)]
	public EditableDouble tractionLimit = 75.0;

	public readonly List<(PartModule, BaseField)> wheelbases = new List<(PartModule, BaseField)>();

	[ValueInfoItem("#MechJeb_Headingerror", InfoItem.Category.Rover, format = "F1", units = "º")]
	public double headingErr;

	[ValueInfoItem("#MechJeb_Speederror", InfoItem.Category.Rover, format = "SI", units = "m/s")]
	public double speedErr;

	public double tgtSpeed;

	public readonly MovingAverage etaSpeed = new MovingAverage(50);

	private double lastETA;

	private float lastThrottle;

	private double curSpeed;

	public override void OnStart(StartState state)
	{
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		headingPID = new PIDController(hPIDp, hPIDi, hPIDd);
		speedPID = new PIDController(sPIDp, sPIDi, sPIDd);
		if (HighLogic.LoadedSceneIsFlight && base.Orbit != null)
		{
			lastBody = base.Orbit.referenceBody;
		}
		GameEvents.onVesselWasModified.Add((OnEvent<Vessel>)OnVesselModified);
		base.OnStart(state);
	}

	public void OnVesselModified(Vessel v)
	{
		try
		{
			wheelbases.Clear();
			wheelbases.AddRange(base.Vessel.Parts.Where((Part p) => p.HasModule<ModuleWheelBase>() && (int)p.GetModule<ModuleWheelBase>().wheelType != 2).Select(delegate(Part p)
			{
				PartModule module2 = p.Modules.GetModule("ModuleWheelBase");
				return (pm: module2, ((BaseFieldList<BaseField, KSPField>)(object)module2.Fields)["isGrounded"]);
			}));
			wheelbases.AddRange(base.Vessel.Parts.Where((Part p) => p.Modules.Contains("KSPWheelBase") && p.Modules.Contains("KSPWheelRotation")).Select(delegate(Part p)
			{
				PartModule module = p.Modules.GetModule("KSPWheelBase");
				return (pm: module, ((BaseFieldList<BaseField, KSPField>)(object)module.Fields)["grounded"]);
			}));
		}
		catch (Exception)
		{
		}
	}

	public double HeadingToPos(Vector3 fromPos, Vector3 toPos)
	{
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_000e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_0039: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		Transform transform = ((Component)base.MainBody).transform;
		Vector3 val = fromPos - transform.position;
		((Vector3)(ref val)).Normalize();
		Vector3 val2 = Vector3.ProjectOnPlane(transform.up, val);
		Vector3 val3 = Vector3.ProjectOnPlane(toPos - fromPos, val);
		return Vector3.SignedAngle(val2, val3, val);
	}

	public float TurningSpeed(double speed, double error)
	{
		return (float)Math.Max(speed / ((Math.Abs(error) / 3.0 > 1.0) ? (Math.Abs(error) / 3.0) : 1.0), turnSpeed);
	}

	public void CalculateTraction()
	{
		if (wheelbases.Count == 0)
		{
			OnVesselModified(base.Vessel);
		}
		traction = 0f;
		foreach (var wheelbasis in wheelbases)
		{
			var (val, _) = wheelbasis;
			if (((BaseField<KSPField>)(object)wheelbasis.Item2).GetValue<bool>((object)val))
			{
				traction += 100f;
			}
		}
		traction /= wheelbases.Count;
	}

	protected override void OnModuleDisabled()
	{
		if (Core.Attitude.Users.Contains(this))
		{
			Core.Attitude.attitudeDeactivate();
			Core.Attitude.Users.Remove(this);
		}
		base.OnModuleDisabled();
	}

	private float Square(float number)
	{
		return number * number;
	}

	private double Square(double number)
	{
		return number * number;
	}

	public override void Drive(FlightCtrlState s)
	{
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_0497: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_00dc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_0718: Unknown result type (might be due to invalid IL or missing references)
		//IL_0723: Unknown result type (might be due to invalid IL or missing references)
		//IL_0733: Unknown result type (might be due to invalid IL or missing references)
		//IL_0738: Unknown result type (might be due to invalid IL or missing references)
		//IL_0743: Unknown result type (might be due to invalid IL or missing references)
		//IL_0751: Unknown result type (might be due to invalid IL or missing references)
		//IL_0756: Unknown result type (might be due to invalid IL or missing references)
		//IL_075b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0766: Unknown result type (might be due to invalid IL or missing references)
		//IL_076b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0770: Unknown result type (might be due to invalid IL or missing references)
		//IL_078a: Unknown result type (might be due to invalid IL or missing references)
		//IL_078f: Unknown result type (might be due to invalid IL or missing references)
		//IL_05ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_05fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_015f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0165: Unknown result type (might be due to invalid IL or missing references)
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		//IL_07e8: Unknown result type (might be due to invalid IL or missing references)
		//IL_07f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0806: Unknown result type (might be due to invalid IL or missing references)
		//IL_0811: Unknown result type (might be due to invalid IL or missing references)
		//IL_081d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0822: Unknown result type (might be due to invalid IL or missing references)
		//IL_07db: Unknown result type (might be due to invalid IL or missing references)
		//IL_0827: Unknown result type (might be due to invalid IL or missing references)
		//IL_082c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0837: Unknown result type (might be due to invalid IL or missing references)
		//IL_0839: Unknown result type (might be due to invalid IL or missing references)
		//IL_083b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0840: Unknown result type (might be due to invalid IL or missing references)
		//IL_0868: Unknown result type (might be due to invalid IL or missing references)
		//IL_086a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0237: Unknown result type (might be due to invalid IL or missing references)
		//IL_023c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0242: Unknown result type (might be due to invalid IL or missing references)
		//IL_0247: Unknown result type (might be due to invalid IL or missing references)
		//IL_03ca: Unknown result type (might be due to invalid IL or missing references)
		//IL_035f: Unknown result type (might be due to invalid IL or missing references)
		if ((Object)(object)base.Orbit.referenceBody != (Object)(object)lastBody)
		{
			WaypointIndex = -1;
			Waypoints.Clear();
		}
		MechJebWaypoint mechJebWaypoint = ((WaypointIndex > -1 && WaypointIndex < Waypoints.Count) ? Waypoints[WaypointIndex] : null);
		bool flag = base.Vessel.ActionGroups[(KSPActionGroup)32];
		curSpeed = Vector3d.Dot(base.VesselState.SurfaceVelocity, base.VesselState.Forward);
		CalculateTraction();
		speedIntAcc = speedPID.INTAccum;
		if (mechJebWaypoint != null && (Object)(object)mechJebWaypoint.Body == (Object)(object)base.Orbit.referenceBody)
		{
			if (ControlHeading)
			{
				double num = Math.Round(HeadingToPos(base.Vessel.CoM, Vector3d.op_Implicit(mechJebWaypoint.Position)), 1);
				if (num != (double)heading)
				{
					heading.Val = num;
				}
			}
			if (ControlSpeed)
			{
				MechJebWaypoint mechJebWaypoint2 = ((WaypointIndex < Waypoints.Count - 1) ? Waypoints[WaypointIndex + 1] : (LoopWaypoints ? Waypoints[0] : null));
				float num2 = Vector3.Distance(base.Vessel.CoM, Vector3d.op_Implicit(mechJebWaypoint.Position));
				if ((Object)(object)mechJebWaypoint.Target != (Object)null)
				{
					num2 += (float)(mechJebWaypoint.Target.srfSpeed * curSpeed) / 2f;
				}
				double val = ((mechJebWaypoint.MaxSpeed > 0f) ? ((EditableDouble)mechJebWaypoint.MaxSpeed) : speed);
				double num3 = ((mechJebWaypoint.MinSpeed > 0f) ? ((double)mechJebWaypoint.MinSpeed) : ((mechJebWaypoint2 != null) ? ((double)TurningSpeed((mechJebWaypoint2.MaxSpeed > 0f) ? ((EditableDouble)mechJebWaypoint2.MaxSpeed) : speed, MuUtils.ClampDegrees180((double)heading - HeadingToPos(Vector3d.op_Implicit(mechJebWaypoint.Position), Vector3d.op_Implicit(mechJebWaypoint2.Position))))) : ((num2 - mechJebWaypoint.Radius > 50f) ? turnSpeed.Val : 1.0)));
				double num4 = Math.Min(val, Math.Max(val2: mechJebWaypoint.Quicksave ? 1.0 : num3, val1: (double)(num2 - mechJebWaypoint.Radius) / curSpeed));
				num4 = ((num4 > (double)turnSpeed) ? ((double)TurningSpeed(num4, headingErr)) : num4);
				float num5 = Math.Max(mechJebWaypoint.Radius, 10f);
				if (num2 < num5)
				{
					if (WaypointIndex + 1 >= Waypoints.Count)
					{
						num4 = new double[2]
						{
							num4,
							(!((double)num2 < (double)num5 * 0.8)) ? 1 : 0
						}.Min();
						if (LoopWaypoints)
						{
							WaypointIndex = 0;
						}
						else
						{
							num4 = 0.0;
							flag = true;
							if (curSpeed < (double)brakeSpeedLimit)
							{
								if (mechJebWaypoint.Quicksave)
								{
									if ((int)FlightGlobals.ClearToSave() == 0)
									{
										WaypointIndex = -1;
										ControlHeading = (ControlSpeed = false);
										QuickSaveLoad.QuickSave();
									}
								}
								else
								{
									WaypointIndex = -1;
									ControlHeading = (ControlSpeed = false);
								}
							}
						}
					}
					else if (mechJebWaypoint.Quicksave)
					{
						num4 = 0.0;
						if (curSpeed < (double)brakeSpeedLimit && (int)FlightGlobals.ClearToSave() == 0)
						{
							WaypointIndex++;
							QuickSaveLoad.QuickSave();
						}
					}
					else
					{
						WaypointIndex++;
					}
				}
				flag = flag || ((s.wheelThrottle == 0f || !base.Vessel.isActiveVessel) && curSpeed < (double)brakeSpeedLimit && num4 < (double)brakeSpeedLimit);
				tgtSpeed = ((num4 >= 0.0) ? num4 : 0.0);
			}
		}
		if (ControlHeading)
		{
			headingPID.INTAccum = Mathf.Clamp((float)headingPID.INTAccum, -1f, 1f);
			double num6 = ((Quaternion)(ref base.VesselState.RotationVesselSurface)).eulerAngles.y;
			headingErr = MuUtils.ClampDegrees180(num6 - (double)heading);
			if (s.wheelSteer == s.wheelSteerTrim || (Object)(object)FlightGlobals.ActiveVessel != (Object)(object)base.Vessel)
			{
				float num7 = ((Math.Abs(curSpeed) > (double)turnSpeed) ? Mathf.Clamp((float)(((double)turnSpeed + 6.0) / Square(curSpeed)), 0.1f, 1f) : 1f);
				double num8 = headingPID.Compute(headingErr);
				if ((double)traction >= (double)tractionLimit)
				{
					s.wheelSteer = Mathf.Clamp((float)num8, 0f - num7, num7);
				}
			}
		}
		if (BrakeOnEject && (Object)(object)base.Vessel.GetReferenceTransformPart() == (Object)null)
		{
			s.wheelThrottle = 0f;
			flag = true;
		}
		else if (ControlSpeed)
		{
			speedPID.INTAccum = Mathf.Clamp((float)speedPID.INTAccum, -5f, 5f);
			speedErr = ((WaypointIndex == -1) ? speed.Val : tgtSpeed) - Vector3d.Dot(base.VesselState.SurfaceVelocity, base.VesselState.Forward);
			if (s.wheelThrottle == s.wheelThrottleTrim || (Object)(object)FlightGlobals.ActiveVessel != (Object)(object)base.Vessel)
			{
				float num9 = (float)speedPID.Compute(speedErr);
				s.wheelThrottle = Mathf.Clamp(num9, -1f, 1f);
				if (curSpeed < 0.0 && s.wheelThrottle < 0f)
				{
					s.wheelThrottle = 0f;
				}
				if (Mathf.Sign(num9) + Mathf.Sign(s.wheelThrottle) == 0f)
				{
					s.wheelThrottle = Mathf.Clamp(num9, -1f, 1f);
				}
				if (speedErr < -1.0 && StabilityControl && Mathf.Sign(s.wheelThrottle) + (float)Math.Sign(curSpeed) == 0f)
				{
					flag = true;
				}
				lastThrottle = Mathf.Clamp(s.wheelThrottle, -1f, 1f);
			}
		}
		if (StabilityControl)
		{
			RaycastHit val3 = default(RaycastHit);
			Physics.Raycast(Vector3d.op_Implicit(base.Vessel.CoM + base.VesselState.SurfaceVelocity * (double)terrainLookAhead + base.VesselState.Up * 100.0), Vector3d.op_Implicit(-base.VesselState.Up), ref val3, 500f, 32768, (QueryTriggerInteraction)1);
			Vector3 normal = ((RaycastHit)(ref val3)).normal;
			if (!Core.Attitude.Users.Contains(this))
			{
				Core.Attitude.Users.Add(this);
			}
			float num10 = (float)curSpeed;
			Vector3 val4 = Vector3d.op_Implicit((traction > 0f) ? (base.VesselState.Forward * 4.0 - ((Component)base.Vessel).transform.right * s.wheelSteer * Mathf.Sign(num10)) : base.VesselState.SurfaceVelocity);
			Vector3.OrthoNormalize(ref normal, ref val4);
			Quaternion val5 = Quaternion.LookRotation(val4, normal);
			if (((Vector3d)(ref base.VesselState.TorqueAvailable)).sqrMagnitude > 0.0)
			{
				Core.Attitude.attitudeTo(QuaternionD.op_Implicit(val5), AttitudeReference.INERTIAL, this);
			}
		}
		if (BrakeOnEnergyDepletion)
		{
			List<Part> source = base.Vessel.Parts.FindAll((Part p) => p.Resources.Contains(PartResourceLibrary.ElectricityHashcode) && p.Resources.Get(PartResourceLibrary.ElectricityHashcode).flowState);
			double num11 = source.Sum((Part p) => p.Resources.Get(PartResourceLibrary.ElectricityHashcode).amount) / source.Sum((Part p) => p.Resources.Get(PartResourceLibrary.ElectricityHashcode).maxAmount);
			bool flag2 = base.Vessel.mainBody.atmosphere && base.Vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().FindAll((ModuleDeployableSolarPanel p) => ((ModuleDeployablePart)p).isBreakable && (int)((ModuleDeployablePart)p).deployState != 4 && (int)((ModuleDeployablePart)p).deployState > 0).Count > 0;
			if (flag2 && num11 > 0.99)
			{
				base.Vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().FindAll((ModuleDeployableSolarPanel p) => ((ModuleDeployablePart)p).isBreakable && (int)((ModuleDeployablePart)p).deployState == 1).ForEach(delegate(ModuleDeployableSolarPanel p)
				{
					((ModuleDeployablePart)p).Retract();
				});
			}
			if (num11 < 0.05 && Math.Sign(s.wheelThrottle) + Math.Sign(curSpeed) != 0)
			{
				s.wheelThrottle = 0f;
			}
			if (flag2 || num11 < 0.03)
			{
				tgtSpeed = 0.0;
			}
			if (curSpeed < (double)brakeSpeedLimit && (num11 < 0.05 || flag2))
			{
				flag = true;
			}
			if (curSpeed < 0.1 && num11 < 0.05 && !waitingForDaylight && base.Vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().FindAll((ModuleDeployableSolarPanel p) => (int)((ModuleDeployablePart)p).deployState == 1).Count > 0)
			{
				waitingForDaylight = true;
			}
		}
		if (s.wheelThrottle != 0f && (Math.Sign(s.wheelThrottle) + Math.Sign(curSpeed) != 0 || curSpeed < 1.0))
		{
			flag = false;
		}
		if (base.Vessel.isActiveVessel)
		{
			if (GameSettings.BRAKES.GetKeyUp(false))
			{
				flag = false;
			}
			if (GameSettings.BRAKES.GetKey(false))
			{
				flag = true;
			}
		}
		tractionLimit = Mathf.Clamp((float)(double)tractionLimit, 0f, 100f);
		base.Vessel.ActionGroups.SetGroup((KSPActionGroup)32, flag && (!StabilityControl || (!ControlHeading && !ControlSpeed) || (double)traction >= (double)tractionLimit));
		if (flag && curSpeed < 0.1)
		{
			s.wheelThrottle = 0f;
		}
	}

	public override void OnFixedUpdate()
	{
		if (!Core.GetComputerModule<MechJebModuleWaypointWindow>().Enabled)
		{
			Waypoints.ForEach(delegate(MechJebWaypoint wp)
			{
				wp.Update();
			});
		}
		if (base.Orbit != null && (Object)(object)lastBody != (Object)(object)base.Orbit.referenceBody)
		{
			lastBody = base.Orbit.referenceBody;
		}
		headingPID.Kp = hPIDp;
		headingPID.Ki = hPIDi;
		headingPID.Kd = hPIDd;
		speedPID.Kp = sPIDp;
		speedPID.Ki = sPIDi;
		speedPID.Kd = sPIDd;
		if (lastETA + 0.2 < DateTime.Now.TimeOfDay.TotalSeconds)
		{
			etaSpeed.Value = curSpeed;
			lastETA = DateTime.Now.TimeOfDay.TotalSeconds;
		}
		if (!Core.GetComputerModule<MechJebModuleRoverWindow>().Enabled)
		{
			Core.GetComputerModule<MechJebModuleRoverWindow>().OnUpdate();
		}
	}

	public override void OnUpdate()
	{
		if (WarpToDaylight && waitingForDaylight && base.Vessel.isActiveVessel)
		{
			List<Part> source = base.Vessel.Parts.FindAll((Part p) => p.Resources.Contains(PartResourceLibrary.ElectricityHashcode) && p.Resources.Get(PartResourceLibrary.ElectricityHashcode).flowState);
			double num = source.Sum((Part p) => p.Resources.Get(PartResourceLibrary.ElectricityHashcode).amount) / source.Sum((Part p) => p.Resources.Get(PartResourceLibrary.ElectricityHashcode).maxAmount);
			if (waitingForDaylight)
			{
				if (base.Vessel.FindPartModulesImplementing<ModuleDeployableSolarPanel>().FindAll((ModuleDeployableSolarPanel p) => (int)((ModuleDeployablePart)p).deployState == 1).Count == 0)
				{
					waitingForDaylight = false;
				}
				Core.Warp.WarpRegularAtRate((num < 0.9) ? 1000 : 50);
				if (num > 0.99)
				{
					waitingForDaylight = false;
					Core.Warp.MinimumWarp();
				}
			}
		}
		else if (!WarpToDaylight && waitingForDaylight)
		{
			waitingForDaylight = false;
		}
		if (!Core.GetComputerModule<MechJebModuleRoverWindow>().Enabled)
		{
			Core.GetComputerModule<MechJebModuleRoverWindow>().OnUpdate();
		}
		if (!StabilityControl && Core.Attitude.Users.Contains(this))
		{
			Core.Attitude.attitudeDeactivate();
			Core.Attitude.Users.Remove(this);
		}
	}

	public override void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnLoad(local, type, global);
		if (local == null)
		{
			return;
		}
		ConfigNode node = local.GetNode("Waypoints");
		if (node != null && node.HasNode("Waypoint"))
		{
			int.TryParse(node.GetValue("Index"), out WaypointIndex);
			Waypoints.Clear();
			ConfigNode[] nodes = node.GetNodes("Waypoint");
			foreach (ConfigNode node2 in nodes)
			{
				Waypoints.Add(new MechJebWaypoint(node2));
			}
		}
	}

	public override void OnSave(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		base.OnSave(local, type, global);
		if (local == null)
		{
			return;
		}
		if (local.HasNode("Waypoints"))
		{
			local.RemoveNode("Waypoints");
		}
		if (Waypoints.Count <= 0)
		{
			return;
		}
		ConfigNode val = local.AddNode("Waypoints");
		val.AddValue("Index", WaypointIndex);
		foreach (MechJebWaypoint waypoint in Waypoints)
		{
			val.AddNode(waypoint.ToConfigNode());
		}
	}

	public MechJebModuleRoverController(MechJebCore core)
		: base(core)
	{
	}
}
