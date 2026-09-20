using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleDockingAutopilot : ComputerModule
{
	public enum DockingStep
	{
		INIT,
		WRONG_SIDE_BACKING_UP,
		WRONG_SIDE_LATERAL,
		WRONG_SIDE_SWITCHSIDE,
		BACKING_UP,
		MOVING_TO_START,
		DOCKING,
		OFF
	}

	public struct Box3d
	{
		public Vector3 center;

		public Vector3 size;
	}

	public string status = "";

	[Persistent(pass = 6)]
	[EditableInfoItem("#MechJeb_DockingSpeedLimit", InfoItem.Category.Thrust, rightLabel = "m/s")]
	public EditableDouble speedLimit = 1.0;

	[Persistent(pass = 1)]
	public readonly EditableDouble rol = new EditableDouble(0.0);

	[Persistent(pass = 1)]
	public bool forceRol;

	[EditableInfoItem("#MechJeb_DockingSpeedLimit", InfoItem.Category.Thrust, rightLabel = "m/s")]
	public EditableDouble overridenSafeDistance = 5.0;

	[Persistent(pass = 1)]
	public bool overrideSafeDistance;

	[Persistent(pass = 1)]
	public bool overrideTargetSize;

	[EditableInfoItem("#MechJeb_DockingSpeedLimit", InfoItem.Category.Thrust, rightLabel = "m/s")]
	public EditableDouble overridenTargetSize = 10.0;

	public float safeDistance = 10f;

	public float targetSize = 5f;

	public bool drawBoundingBox;

	public DockingStep dockingStep = DockingStep.OFF;

	private Vector3d zAxis;

	public double zSep;

	public Vector3d lateralSep;

	public double relativeZ;

	public double relativeLateral;

	private ITargetable lastTarget;

	private const float dockingcorridorRadius = 1f;

	private double acquireRange = 0.25;

	public Box3d vesselBoundingBox;

	public Box3d targetBoundingBox;

	public MechJebModuleDockingAutopilot(MechJebCore core)
		: base(core)
	{
	}

	public override void OnStart(StartState state)
	{
		//IL_0000: Unknown result type (might be due to invalid IL or missing references)
		//IL_0003: Unknown result type (might be due to invalid IL or missing references)
		//IL_0005: Invalid comparison between Unknown and I4
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Expected O, but got Unknown
		if ((int)state == 0 || (int)state == 1)
		{
			return;
		}
		Core.AddToPostDrawQueue(new Callback(DrawBoundingBox));
		GameEvents.onPartCouple.Add((OnEvent<FromToAction<Part, Part>>)delegate(FromToAction<Part, Part> ev)
		{
			//IL_0009: Unknown result type (might be due to invalid IL or missing references)
			//IL_0021: Unknown result type (might be due to invalid IL or missing references)
			if (dockingStep != DockingStep.OFF && ((Object)(object)ev.from.vessel == (Object)(object)base.Vessel || (Object)(object)ev.to.vessel == (Object)(object)base.Vessel))
			{
				EndDocking();
			}
		});
	}

	protected override void OnModuleEnabled()
	{
		Core.RCS.Users.Add(this);
		Core.Attitude.Users.Add(this);
		dockingStep = DockingStep.INIT;
	}

	protected override void OnModuleDisabled()
	{
		Core.RCS.Users.Remove(this);
		Core.Attitude.attitudeDeactivate();
		dockingStep = DockingStep.OFF;
		drawBoundingBox = false;
	}

	private double FixSpeed(double s)
	{
		if ((double)speedLimit != 0.0)
		{
			if (s > (double)speedLimit)
			{
				s = speedLimit;
			}
			if (s < 0.0 - (double)speedLimit)
			{
				s = 0.0 - (double)speedLimit;
			}
		}
		return s;
	}

	private double MaxSpeedForDistance(double distance, Vector3d axis)
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_001b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		Vector3d direction = Vector3d.op_Implicit(base.Vessel.ReferenceTransform.InverseTransformDirection(Vector3d.op_Implicit(axis)));
		return FixSpeed(Math.Sqrt(2.0 * Math.Abs(distance) * base.VesselState.RCSThrustAvailable.GetMagnitude(direction) * Core.RCS.rcsAccelFactor() / base.VesselState.Mass));
	}

	public override void Drive(FlightCtrlState s)
	{
		//IL_003b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		//IL_007b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0080: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00da: Unknown result type (might be due to invalid IL or missing references)
		//IL_018a: Unknown result type (might be due to invalid IL or missing references)
		//IL_018f: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_01db: Unknown result type (might be due to invalid IL or missing references)
		//IL_042a: Unknown result type (might be due to invalid IL or missing references)
		//IL_043a: Unknown result type (might be due to invalid IL or missing references)
		//IL_043f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0444: Unknown result type (might be due to invalid IL or missing references)
		//IL_0449: Unknown result type (might be due to invalid IL or missing references)
		//IL_02bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_04c5: Unknown result type (might be due to invalid IL or missing references)
		//IL_046d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0472: Unknown result type (might be due to invalid IL or missing references)
		//IL_0477: Unknown result type (might be due to invalid IL or missing references)
		//IL_047c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0481: Unknown result type (might be due to invalid IL or missing references)
		//IL_0494: Unknown result type (might be due to invalid IL or missing references)
		//IL_0499: Unknown result type (might be due to invalid IL or missing references)
		//IL_049e: Unknown result type (might be due to invalid IL or missing references)
		//IL_04a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_04a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_04d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_04de: Unknown result type (might be due to invalid IL or missing references)
		//IL_04e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_04eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_04f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_04f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_04fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_0507: Unknown result type (might be due to invalid IL or missing references)
		//IL_0508: Unknown result type (might be due to invalid IL or missing references)
		//IL_050a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0514: Unknown result type (might be due to invalid IL or missing references)
		//IL_0516: Unknown result type (might be due to invalid IL or missing references)
		//IL_0526: Unknown result type (might be due to invalid IL or missing references)
		//IL_052b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0530: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Target.NormalTargetExists)
		{
			EndDocking();
		}
		else
		{
			if (dockingStep == DockingStep.OFF || dockingStep == DockingStep.INIT)
			{
				return;
			}
			Vector3d vel = Core.Target.TargetOrbit.GetVel();
			double num = MaxSpeedForDistance(Math.Max(zSep - acquireRange, 0.0), -zAxis);
			double num2 = MaxSpeedForDistance(((Vector3d)(ref lateralSep)).magnitude, -lateralSep);
			bool flag = true;
			switch (dockingStep)
			{
			case DockingStep.WRONG_SIDE_BACKING_UP:
				num = MaxSpeedForDistance((double)safeDistance + zSep + 2.0, -zAxis);
				if (((Vector3d)(ref lateralSep)).magnitude < (double)safeDistance)
				{
					num2 *= -1.0;
				}
				else if (((Vector3d)(ref lateralSep)).magnitude < (double)(safeDistance * 2f))
				{
					num2 = 0.0;
				}
				flag = false;
				status = Localizer.Format("#MechJeb_Docking_status1", new string[2]
				{
					num.ToString("F2"),
					num2.ToString()
				});
				break;
			case DockingStep.WRONG_SIDE_LATERAL:
				num = 0.0;
				num2 = 0.0 - MaxSpeedForDistance((double)safeDistance - ((Vector3d)(ref lateralSep)).magnitude + 2.0, -lateralSep);
				status = Localizer.Format("#MechJeb_Docking_status2", new string[1] { num2.ToString("F2") });
				break;
			case DockingStep.WRONG_SIDE_SWITCHSIDE:
				num = 0.0 - MaxSpeedForDistance(0.0 - zSep + (double)targetSize, -zAxis);
				if (((Vector3d)(ref lateralSep)).magnitude < (double)safeDistance)
				{
					num2 *= -1.0;
				}
				else if (((Vector3d)(ref lateralSep)).magnitude < (double)(safeDistance * 2f))
				{
					num2 = 0.0;
				}
				status = Localizer.Format("#MechJeb_Docking_status3", new string[2]
				{
					num.ToString("F2"),
					num2.ToString()
				});
				break;
			case DockingStep.BACKING_UP:
				if (((Vector3d)(ref lateralSep)).magnitude < (double)safeDistance)
				{
					num2 *= -1.0;
				}
				else if (((Vector3d)(ref lateralSep)).magnitude < (double)(safeDistance * 2f))
				{
					num2 = 0.0;
				}
				num = 0.0 - MaxSpeedForDistance((double)(1f + targetSize) - zSep, -zAxis);
				flag = false;
				status = Localizer.Format("#MechJeb_Docking_status4", new string[1] { num.ToString("F2") });
				break;
			case DockingStep.MOVING_TO_START:
				num = ((!(zSep < (double)safeDistance)) ? 0.0 : (num * -1.0));
				status = Localizer.Format("#MechJeb_Docking_status5", new string[1] { num.ToString("F2") });
				break;
			case DockingStep.DOCKING:
			{
				double num3 = Math.Abs(((Vector3d)(ref lateralSep)).magnitude / num2);
				double num4 = Math.Abs(zSep / num);
				if ((zSep <= ((Vector3d)(ref lateralSep)).magnitude * 10.0 || num4 <= num3 * 10.0) && num3 > 0.0 && num4 > 0.0)
				{
					num *= Math.Min(num4 / num3, 1.0);
					num2 = FixSpeed(num2 * 2.0);
				}
				status = Localizer.Format("#MechJeb_Docking_status6", new string[2]
				{
					num.ToString("F2"),
					num2.ToString("F2")
				});
				break;
			}
			}
			if (!flag)
			{
				Core.Attitude.attitudeTo(QuaternionD.op_Implicit(Quaternion.LookRotation(base.Vessel.GetTransform().up, -base.Vessel.GetTransform().forward)), AttitudeReference.INERTIAL, this);
			}
			else if (forceRol)
			{
				Core.Attitude.attitudeTo(QuaternionD.op_Implicit(Quaternion.LookRotation(Vector3d.op_Implicit(Vector3d.back), Vector3d.op_Implicit(Vector3d.up)) * Quaternion.AngleAxis(0f - (float)(double)rol, Vector3d.op_Implicit(Vector3d.back))), AttitudeReference.TARGET_ORIENTATION, this);
			}
			else
			{
				Core.Attitude.attitudeTo(Vector3d.back, AttitudeReference.TARGET_ORIENTATION, this);
			}
			Vector3d val = -((Vector3d)(ref lateralSep)).normalized * num2 + num * zAxis;
			Core.RCS.SetTargetWorldVelocity(vel + val);
			MechJebModuleDebugArrows.debugVector = val;
			MechJebModuleDebugArrows.debugVector2 = -Core.Target.RelativePosition;
		}
	}

	public override void OnFixedUpdate()
	{
		if (!Core.Target.NormalTargetExists)
		{
			EndDocking();
			return;
		}
		if (!overrideTargetSize)
		{
			targetSize = ((Vector3)(ref targetBoundingBox.size)).magnitude;
		}
		else
		{
			targetSize = (float)overridenTargetSize.Val;
		}
		if (!overrideSafeDistance)
		{
			safeDistance = ((Vector3)(ref vesselBoundingBox.size)).magnitude + targetSize + 0.5f;
		}
		else
		{
			safeDistance = (float)overridenSafeDistance.Val;
		}
		UpdateDistance();
		switch (dockingStep)
		{
		case DockingStep.INIT:
			InitDocking();
			break;
		case DockingStep.WRONG_SIDE_BACKING_UP:
			if (0.0 - zSep > (double)safeDistance)
			{
				dockingStep = DockingStep.WRONG_SIDE_LATERAL;
			}
			break;
		case DockingStep.WRONG_SIDE_LATERAL:
			if (((Vector3d)(ref lateralSep)).magnitude > (double)safeDistance)
			{
				dockingStep = DockingStep.WRONG_SIDE_SWITCHSIDE;
			}
			break;
		case DockingStep.WRONG_SIDE_SWITCHSIDE:
			if (zSep > 0.0)
			{
				dockingStep = DockingStep.BACKING_UP;
			}
			break;
		case DockingStep.BACKING_UP:
			if (zSep > (double)targetSize)
			{
				dockingStep = DockingStep.MOVING_TO_START;
			}
			break;
		case DockingStep.MOVING_TO_START:
			if (((Vector3d)(ref lateralSep)).magnitude < 1.0 && zSep >= (double)targetSize)
			{
				dockingStep = DockingStep.DOCKING;
			}
			break;
		case DockingStep.DOCKING:
			if (zSep < acquireRange)
			{
				EndDocking();
			}
			else if (((Vector3d)(ref lateralSep)).magnitude > 1.0)
			{
				if (zSep < 0.0)
				{
					dockingStep = DockingStep.WRONG_SIDE_BACKING_UP;
				}
				else if (((Vector3d)(ref lateralSep)).magnitude > 1.0 && zSep < 1.0)
				{
					dockingStep = DockingStep.MOVING_TO_START;
				}
			}
			break;
		}
	}

	private void UpdateDistance()
	{
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0010: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0049: Unknown result type (might be due to invalid IL or missing references)
		//IL_004e: Unknown result type (might be due to invalid IL or missing references)
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_0065: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_007c: Unknown result type (might be due to invalid IL or missing references)
		//IL_008c: Unknown result type (might be due to invalid IL or missing references)
		Vector3d relativePosition = Core.Target.RelativePosition;
		Vector3 dockingAxis = Core.Target.DockingAxis;
		zAxis = Vector3d.op_Implicit(((Vector3)(ref dockingAxis)).normalized);
		zSep = 0.0 - Vector3d.Dot(relativePosition, zAxis);
		lateralSep = Vector3d.Exclude(zAxis, relativePosition);
		relativeZ = Vector3d.Dot(Core.Target.RelativeVelocity, zAxis);
		relativeLateral = Vector3d.Dot(lateralSep, Core.Target.RelativeVelocity);
	}

	private void InitDocking()
	{
		//IL_00d8: Unknown result type (might be due to invalid IL or missing references)
		lastTarget = Core.Target.Target;
		try
		{
			vesselBoundingBox = base.Vessel.GetBoundingBox();
			targetBoundingBox = lastTarget.GetVessel().GetBoundingBox();
			if (!overrideTargetSize)
			{
				targetSize = ((Vector3)(ref targetBoundingBox.size)).magnitude;
			}
			else
			{
				targetSize = (float)overridenTargetSize.Val;
			}
			if (!overrideSafeDistance)
			{
				safeDistance = ((Vector3)(ref vesselBoundingBox.size)).magnitude + targetSize + 0.5f;
			}
			else
			{
				safeDistance = (float)overridenSafeDistance.Val;
			}
			if (Core.Target.Target is ModuleDockingNode)
			{
				acquireRange = (double)((ModuleDockingNode)Core.Target.Target).acquireRange * 0.5;
			}
			else
			{
				acquireRange = 0.25;
			}
		}
		catch (Exception message)
		{
			ComputerModule.Print(message);
		}
		if (zSep < 0.0)
		{
			if (Math.Abs(zSep) > (double)(((Vector3)(ref vesselBoundingBox.size)).magnitude * 0.5f))
			{
				dockingStep = DockingStep.WRONG_SIDE_BACKING_UP;
			}
			else
			{
				dockingStep = DockingStep.BACKING_UP;
			}
		}
		else if (((Vector3d)(ref lateralSep)).magnitude > 1.0)
		{
			if (zSep < (double)targetSize)
			{
				dockingStep = DockingStep.BACKING_UP;
			}
			else
			{
				dockingStep = DockingStep.MOVING_TO_START;
			}
		}
		else
		{
			dockingStep = DockingStep.DOCKING;
		}
	}

	private void EndDocking()
	{
		dockingStep = DockingStep.OFF;
		Users.Clear();
		base.Enabled = false;
	}

	private void DrawBoundingBox()
	{
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		if (drawBoundingBox && (Object)(object)base.Vessel == (Object)(object)FlightGlobals.ActiveVessel)
		{
			vesselBoundingBox = base.Vessel.GetBoundingBox();
			GLUtils.DrawBoundingBox(base.Vessel.mainBody, base.Vessel, vesselBoundingBox, Color.green);
			if (Core.Target.Target != null)
			{
				Vessel vessel = Core.Target.Target.GetVessel();
				targetBoundingBox = vessel.GetBoundingBox();
				GLUtils.DrawBoundingBox(vessel.mainBody, vessel, targetBoundingBox, Color.blue);
			}
		}
	}
}
