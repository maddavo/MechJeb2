using System;
using UnityEngine;

namespace MuMech;

internal class MechJebModuleDebugArrows : ComputerModule
{
	[Persistent(pass = 4)]
	public bool displayAtCoM;

	[Persistent(pass = 4)]
	public bool seeThrough;

	[Persistent(pass = 4)]
	public bool comSphereActive;

	public static DebugIcoSphere comSphere;

	[Persistent(pass = 4)]
	public bool colSphereActive;

	public static DebugIcoSphere colSphere;

	[Persistent(pass = 4)]
	public bool cotSphereActive;

	public static DebugIcoSphere cotSphere;

	[Persistent(pass = 4)]
	public readonly EditableDouble comSphereRadius = new EditableDouble(0.09);

	[Persistent(pass = 4)]
	public bool srfVelocityArrowActive;

	public static DebugArrow srfVelocityArrow;

	[Persistent(pass = 4)]
	public bool obtVelocityArrowActive;

	public static DebugArrow obtVelocityArrow;

	[Persistent(pass = 4)]
	public bool dotArrowActive;

	public static DebugArrow dotArrow;

	[Persistent(pass = 4)]
	public bool forwardArrowActive;

	public static DebugArrow forwardArrow;

	[Persistent(pass = 4)]
	public bool requestedAttitudeArrowActive;

	public static DebugArrow requestedAttitudeArrow;

	[Persistent(pass = 4)]
	public bool debugArrowActive;

	public static DebugArrow debugArrow;

	[Persistent(pass = 4)]
	public bool debugArrow2Active;

	public static DebugArrow debugArrow2;

	public static Vector3d debugVector = Vector3d.one;

	public static Vector3d debugVector2 = Vector3d.one;

	[Persistent(pass = 4)]
	public readonly EditableDouble arrowsLength = new EditableDouble(4.0);

	public MechJebModuleDebugArrows(MechJebCore core)
		: base(core)
	{
		base.Enabled = true;
	}

	public override void OnDestroy()
	{
		if (comSphere != null)
		{
			comSphere.Destroy();
			comSphere = null;
			colSphere.Destroy();
			colSphere = null;
			cotSphere.Destroy();
			cotSphere = null;
			srfVelocityArrow.Destroy();
			srfVelocityArrow = null;
			obtVelocityArrow.Destroy();
			obtVelocityArrow = null;
			dotArrow.Destroy();
			dotArrow = null;
			forwardArrow.Destroy();
			forwardArrow = null;
			requestedAttitudeArrow.Destroy();
			requestedAttitudeArrow = null;
			debugArrow.Destroy();
			debugArrow = null;
			debugArrow2.Destroy();
			debugArrow2 = null;
		}
	}

	public override void OnUpdate()
	{
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_0105: Unknown result type (might be due to invalid IL or missing references)
		//IL_010a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0111: Unknown result type (might be due to invalid IL or missing references)
		//IL_0116: Unknown result type (might be due to invalid IL or missing references)
		//IL_0117: Unknown result type (might be due to invalid IL or missing references)
		//IL_011c: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_003d: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_006d: Unknown result type (might be due to invalid IL or missing references)
		//IL_007d: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_013c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0130: Unknown result type (might be due to invalid IL or missing references)
		//IL_0135: Unknown result type (might be due to invalid IL or missing references)
		//IL_013d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0142: Unknown result type (might be due to invalid IL or missing references)
		//IL_0170: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d5: Unknown result type (might be due to invalid IL or missing references)
		//IL_01da: Unknown result type (might be due to invalid IL or missing references)
		//IL_01db: Unknown result type (might be due to invalid IL or missing references)
		//IL_0244: Unknown result type (might be due to invalid IL or missing references)
		//IL_0249: Unknown result type (might be due to invalid IL or missing references)
		//IL_024a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0297: Unknown result type (might be due to invalid IL or missing references)
		//IL_0298: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_0300: Unknown result type (might be due to invalid IL or missing references)
		//IL_0301: Unknown result type (might be due to invalid IL or missing references)
		//IL_030c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0385: Unknown result type (might be due to invalid IL or missing references)
		//IL_038a: Unknown result type (might be due to invalid IL or missing references)
		//IL_038b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0396: Unknown result type (might be due to invalid IL or missing references)
		//IL_0402: Unknown result type (might be due to invalid IL or missing references)
		//IL_0403: Unknown result type (might be due to invalid IL or missing references)
		//IL_0413: Unknown result type (might be due to invalid IL or missing references)
		//IL_0418: Unknown result type (might be due to invalid IL or missing references)
		//IL_0499: Unknown result type (might be due to invalid IL or missing references)
		//IL_049a: Unknown result type (might be due to invalid IL or missing references)
		//IL_04aa: Unknown result type (might be due to invalid IL or missing references)
		//IL_04af: Unknown result type (might be due to invalid IL or missing references)
		//IL_0517: Unknown result type (might be due to invalid IL or missing references)
		//IL_051c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0521: Unknown result type (might be due to invalid IL or missing references)
		//IL_057e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0583: Unknown result type (might be due to invalid IL or missing references)
		//IL_0584: Unknown result type (might be due to invalid IL or missing references)
		//IL_0589: Unknown result type (might be due to invalid IL or missing references)
		//IL_058a: Unknown result type (might be due to invalid IL or missing references)
		//IL_058f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0596: Unknown result type (might be due to invalid IL or missing references)
		//IL_0597: Unknown result type (might be due to invalid IL or missing references)
		if (!((Object)(object)base.Vessel != (Object)(object)FlightGlobals.ActiveVessel))
		{
			if (comSphere == null)
			{
				comSphere = new DebugIcoSphere(XKCDColors.BloodRed, seeThrough: true);
				colSphere = new DebugIcoSphere(XKCDColors.Teal, seeThrough: true);
				cotSphere = new DebugIcoSphere(XKCDColors.PurplePink, seeThrough: true);
				srfVelocityArrow = new DebugArrow(Color.green);
				obtVelocityArrow = new DebugArrow(Color.red);
				dotArrow = new DebugArrow(XKCDColors.PurplePink);
				forwardArrow = new DebugArrow(XKCDColors.ElectricBlue);
				requestedAttitudeArrow = new DebugArrow(Color.gray);
				debugArrow = new DebugArrow(XKCDColors.Fuchsia);
				debugArrow2 = new DebugArrow(XKCDColors.LightBlue);
			}
			Vector3d val = base.VesselState.OrbitalVelocity - Krakensbane.GetFrameVelocity();
			Vector3d rotFrameVel = base.Vessel.orbit.GetRotFrameVel(base.Vessel.orbit.referenceBody);
			Vector3d val2 = (val - ((Vector3d)(ref rotFrameVel)).xzy) * (double)Time.fixedDeltaTime;
			Vector3d val3 = base.VesselState.CoM + val2;
			Vector3 val4 = Vector3d.op_Implicit(displayAtCoM ? val3 : Vector3d.op_Implicit(base.Vessel.ReferenceTransform.position));
			comSphere.State(comSphereActive && Core.ShowGui);
			if (comSphereActive)
			{
				comSphere.Set(val3);
				comSphere.SetRadius((float)comSphereRadius.Val);
			}
			colSphere.State(colSphereActive && base.VesselState.CoLWeightSum > 0.0 && Core.ShowGui);
			if (colSphereActive)
			{
				colSphere.Set(base.VesselState.CoL + val2);
				colSphere.SetRadius((float)comSphereRadius.Val);
			}
			cotSphere.State(cotSphereActive && base.VesselState.CoTWeightSum > 0.0 && Core.ShowGui);
			if (cotSphereActive)
			{
				cotSphere.Set(base.VesselState.CoT + val2);
				cotSphere.SetRadius((float)comSphereRadius.Val);
			}
			srfVelocityArrow.State(srfVelocityArrowActive && Core.ShowGui);
			if (srfVelocityArrowActive)
			{
				srfVelocityArrow.Set(Vector3d.op_Implicit(val4), base.Vessel.srf_velocity);
				srfVelocityArrow.SetLength((float)arrowsLength.Val);
				srfVelocityArrow.SeeThrough(seeThrough);
			}
			obtVelocityArrow.State(obtVelocityArrowActive && Core.ShowGui);
			if (obtVelocityArrowActive)
			{
				obtVelocityArrow.Set(Vector3d.op_Implicit(val4), base.Vessel.obt_velocity);
				obtVelocityArrow.SetLength((float)arrowsLength.Val);
				obtVelocityArrow.SeeThrough(seeThrough);
			}
			dotArrow.State(dotArrowActive && base.VesselState.ThrustCurrent > 0.0 && Core.ShowGui);
			if (dotArrowActive)
			{
				dotArrow.Set(base.VesselState.CoT + val2, base.VesselState.DoT);
				dotArrow.SetLength((float)Math.Log10(base.VesselState.ThrustCurrent + 1.0));
				dotArrow.SeeThrough(seeThrough);
			}
			forwardArrow.State(forwardArrowActive && Core.ShowGui);
			if (forwardArrowActive)
			{
				forwardArrow.Set(Vector3d.op_Implicit(val4), Vector3d.op_Implicit(base.Vessel.GetTransform().up));
				forwardArrow.SetLength((float)arrowsLength.Val);
				forwardArrow.SeeThrough(seeThrough);
			}
			requestedAttitudeArrow.State(requestedAttitudeArrowActive && Core.Attitude.Enabled && Core.ShowGui);
			if (requestedAttitudeArrowActive && Core.Attitude.Enabled)
			{
				requestedAttitudeArrow.Set(Vector3d.op_Implicit(val4), QuaternionD.op_Implicit(Core.Attitude.RequestedAttitude));
				requestedAttitudeArrow.SetLength((float)arrowsLength.Val);
				requestedAttitudeArrow.SeeThrough(seeThrough);
			}
			debugArrow.State(debugArrowActive && Core.ShowGui);
			if (debugArrowActive)
			{
				debugArrow.Set(Vector3d.op_Implicit(base.Vessel.ReferenceTransform.position), debugVector);
				debugArrow.SetLength((float)((Vector3d)(ref debugVector)).magnitude);
				debugArrow.SeeThrough(seeThrough);
			}
			debugArrow2.State(debugArrow2Active && Core.ShowGui);
			if (debugArrow2Active)
			{
				Vector3d direction = base.VesselState.CoL - val3 + val2;
				debugArrow2.Set(val3, direction);
				debugArrow2.SetLength((float)((Vector3d)(ref direction)).magnitude);
				debugArrow2.SeeThrough(seeThrough);
			}
		}
	}
}
