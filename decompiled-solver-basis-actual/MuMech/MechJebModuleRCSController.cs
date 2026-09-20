using System;
using UnityEngine;

namespace MuMech;

public class MechJebModuleRCSController : ComputerModule
{
	private enum ControlType
	{
		TARGET_VELOCITY,
		VELOCITY_ERROR,
		VELOCITY_TARGET_REL,
		POSITION_TARGET_REL
	}

	public Vector3d targetVelocity = Vector3d.zero;

	public readonly PIDControllerV2 pid;

	private Vector3d lastAct = Vector3d.zero;

	private Vector3d worldVelocityDelta = Vector3d.zero;

	private Vector3d prev_worldVelocityDelta = Vector3d.zero;

	private ControlType controlType;

	[Persistent(pass = 4)]
	[ToggleInfoItem("#MechJeb_conserveFuel", InfoItem.Category.Thrust)]
	public readonly bool conserveFuel;

	[EditableInfoItem("#MechJeb_conserveThreshold", InfoItem.Category.Thrust, rightLabel = "m/s")]
	public readonly EditableDouble conserveThreshold = 0.05;

	[Persistent(pass = 7)]
	[EditableInfoItem("#MechJeb_RCSTf", InfoItem.Category.Thrust)]
	public EditableDouble Tf = 1.0;

	[Persistent(pass = 7)]
	public readonly EditableDouble Kp = 0.125;

	[Persistent(pass = 7)]
	public readonly EditableDouble Ki = 0.07;

	[Persistent(pass = 7)]
	public readonly EditableDouble Kd = 0.53;

	[Persistent(pass = 4)]
	public bool rcsManualPID;

	[Persistent(pass = 4)]
	[ToggleInfoItem("#MechJeb_RCSThrottle", InfoItem.Category.Thrust)]
	public bool rcsThrottle = true;

	[Persistent(pass = 4)]
	[ToggleInfoItem("#MechJeb_rcsForRotation", InfoItem.Category.Thrust)]
	public bool rcsForRotation = true;

	public MechJebModuleRCSController(MechJebCore core)
		: base(core)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		Priority = 600;
		pid = new PIDControllerV2(Kp, Ki, Kd, 1.0, -1.0);
	}

	protected override void OnModuleEnabled()
	{
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		setPIDParameters();
		pid.Reset();
		lastAct = Vector3d.zero;
		worldVelocityDelta = Vector3d.zero;
		prev_worldVelocityDelta = Vector3d.zero;
		controlType = ControlType.VELOCITY_ERROR;
		base.OnModuleEnabled();
	}

	public bool rcsDeactivate()
	{
		Users.Clear();
		return true;
	}

	public void setPIDParameters()
	{
		if (rcsManualPID)
		{
			pid.Kd = Kd;
			pid.Kp = Kp;
			pid.Ki = Ki;
			return;
		}
		Tf = Math.Max(Tf, 0.02);
		pid.Kd = 0.53 / (double)Tf;
		pid.Kp = pid.Kd / (3.0 * Math.Sqrt(2.0) * (double)Tf);
		pid.Ki = pid.Kp / (12.0 * Math.Sqrt(2.0) * (double)Tf);
		Kd.Val = pid.Kd;
		Kp.Val = pid.Kp;
		Ki.Val = pid.Ki;
	}

	[GeneralInfoItem("#MechJeb_RCSPid", InfoItem.Category.Thrust)]
	public void PIDGUI()
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		rcsManualPID = GUILayout.Toggle(rcsManualPID, "RCS Manual Pid", Array.Empty<GUILayoutOption>());
		if (rcsManualPID)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label("P", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			Kp.Text = GUILayout.TextField(Kp.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
			{
				GuiUtils.LayoutExpandWidth,
				GuiUtils.LayoutWidth(60f)
			});
			GUILayout.Label("I", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			Kd.Text = GUILayout.TextField(Kd.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
			{
				GuiUtils.LayoutExpandWidth,
				GuiUtils.LayoutWidth(60f)
			});
			GUILayout.Label("D", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			Ki.Text = GUILayout.TextField(Ki.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
			{
				GuiUtils.LayoutExpandWidth,
				GuiUtils.LayoutWidth(60f)
			});
			GUILayout.EndHorizontal();
		}
		else
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label("Tf", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			Tf.Text = GUILayout.TextField(Tf.Text, (GUILayoutOption[])(object)new GUILayoutOption[2]
			{
				GuiUtils.LayoutExpandWidth,
				GuiUtils.LayoutWidth(40f)
			});
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label("P", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.Label(Kp.Val.ToString("F4"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label("I", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.Label(Ki.Val.ToString("F4"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label("D", (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.Label(Kd.Val.ToString("F4"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.EndHorizontal();
		}
		GUILayout.EndVertical();
		setPIDParameters();
	}

	public double rcsAccelFactor()
	{
		return pid.Kp;
	}

	protected override void OnModuleDisabled()
	{
		base.Vessel.ActionGroups.SetGroup((KSPActionGroup)8, false);
		base.OnModuleDisabled();
	}

	public void SetTargetWorldVelocity(Vector3d vel)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		targetVelocity = vel;
		controlType = ControlType.TARGET_VELOCITY;
	}

	public void SetWorldVelocityError(Vector3d dv)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		worldVelocityDelta = -dv;
		if (controlType != ControlType.VELOCITY_ERROR)
		{
			prev_worldVelocityDelta = worldVelocityDelta;
			controlType = ControlType.VELOCITY_ERROR;
		}
	}

	public void SetTargetRelative(Vector3d vel)
	{
		//IL_0001: Unknown result type (might be due to invalid IL or missing references)
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		targetVelocity = vel;
		controlType = ControlType.VELOCITY_TARGET_REL;
	}

	public override void Drive(FlightCtrlState s)
	{
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_0085: Unknown result type (might be due to invalid IL or missing references)
		//IL_008a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0090: Unknown result type (might be due to invalid IL or missing references)
		//IL_0095: Unknown result type (might be due to invalid IL or missing references)
		//IL_009a: Unknown result type (might be due to invalid IL or missing references)
		//IL_009f: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_0065: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0070: Unknown result type (might be due to invalid IL or missing references)
		//IL_0075: Unknown result type (might be due to invalid IL or missing references)
		//IL_00eb: Unknown result type (might be due to invalid IL or missing references)
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_010b: Unknown result type (might be due to invalid IL or missing references)
		//IL_019f: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ec: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f1: Unknown result type (might be due to invalid IL or missing references)
		//IL_01f6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0228: Unknown result type (might be due to invalid IL or missing references)
		//IL_0229: Unknown result type (might be due to invalid IL or missing references)
		//IL_022a: Unknown result type (might be due to invalid IL or missing references)
		//IL_022f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0231: Unknown result type (might be due to invalid IL or missing references)
		//IL_0232: Unknown result type (might be due to invalid IL or missing references)
		//IL_0238: Unknown result type (might be due to invalid IL or missing references)
		//IL_0254: Unknown result type (might be due to invalid IL or missing references)
		//IL_0270: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_0200: Unknown result type (might be due to invalid IL or missing references)
		//IL_0205: Unknown result type (might be due to invalid IL or missing references)
		//IL_0210: Unknown result type (might be due to invalid IL or missing references)
		//IL_0215: Unknown result type (might be due to invalid IL or missing references)
		//IL_0218: Unknown result type (might be due to invalid IL or missing references)
		//IL_021d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0171: Unknown result type (might be due to invalid IL or missing references)
		//IL_0179: Unknown result type (might be due to invalid IL or missing references)
		//IL_0180: Unknown result type (might be due to invalid IL or missing references)
		//IL_0185: Unknown result type (might be due to invalid IL or missing references)
		//IL_018a: Unknown result type (might be due to invalid IL or missing references)
		setPIDParameters();
		switch (controlType)
		{
		case ControlType.TARGET_VELOCITY:
			worldVelocityDelta = base.VesselState.OrbitalVelocity - targetVelocity;
			break;
		case ControlType.VELOCITY_TARGET_REL:
			if (Core.Target.Target == null)
			{
				rcsDeactivate();
				return;
			}
			worldVelocityDelta = Core.Target.RelativeVelocity - targetVelocity;
			break;
		}
		Vector3d val = Vector3d.op_Implicit(Quaternion.Inverse(base.Vessel.GetTransform().rotation) * Vector3d.op_Implicit(worldVelocityDelta));
		if (!conserveFuel || ((Vector3d)(ref val)).magnitude > (double)conserveThreshold)
		{
			if (!base.Vessel.ActionGroups[(KSPActionGroup)8])
			{
				base.Vessel.ActionGroups.SetGroup((KSPActionGroup)8, true);
			}
			Vector3d val2 = default(Vector3d);
			for (int i = 0; i < Vector6.Values.Length; i++)
			{
				Vector6.Direction direction = Vector6.Values[i];
				double num = Vector3d.Dot(val, Vector6.Directions[(int)direction]);
				double num2 = base.VesselState.RCSThrustAvailable[direction];
				if (num2 > 0.0 && Math.Abs(num) > 0.001)
				{
					double num3 = num / (num2 * (double)TimeWarp.fixedDeltaTime / base.VesselState.Mass);
					if (num3 > 0.0)
					{
						val2 += Vector6.Directions[(int)direction] * num3;
					}
				}
			}
			Vector3d omega = Vector3d.zero;
			switch (controlType)
			{
			case ControlType.TARGET_VELOCITY:
				omega = Vector3d.op_Implicit(Quaternion.Inverse(base.Vessel.GetTransform().rotation) * Vector3d.op_Implicit(base.Vessel.acceleration - base.VesselState.GravityForce));
				break;
			case ControlType.VELOCITY_ERROR:
			case ControlType.VELOCITY_TARGET_REL:
				omega = (worldVelocityDelta - prev_worldVelocityDelta) / (double)TimeWarp.fixedDeltaTime;
				prev_worldVelocityDelta = worldVelocityDelta;
				break;
			}
			val2 = (lastAct = pid.Compute(val2, omega));
			s.X = Mathf.Clamp((float)val2.x, -1f, 1f);
			s.Y = Mathf.Clamp((float)val2.z, -1f, 1f);
			s.Z = Mathf.Clamp((float)val2.y, -1f, 1f);
		}
		else if (conserveFuel && base.Vessel.ActionGroups[(KSPActionGroup)8])
		{
			base.Vessel.ActionGroups.SetGroup((KSPActionGroup)8, false);
		}
		base.Drive(s);
	}
}
