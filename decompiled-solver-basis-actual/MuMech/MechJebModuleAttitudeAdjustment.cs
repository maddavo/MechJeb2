using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleAttitudeAdjustment : DisplayModule
{
	[Persistent(pass = 4)]
	public bool showInfos;

	public MechJebModuleAttitudeAdjustment(MechJebCore core)
		: base(core)
	{
	}

	protected override void WindowGUI(int windowID)
	{
		//IL_018f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0214: Unknown result type (might be due to invalid IL or missing references)
		//IL_0294: Unknown result type (might be due to invalid IL or missing references)
		//IL_02c6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02db: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_032b: Unknown result type (might be due to invalid IL or missing references)
		//IL_03a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_0428: Unknown result type (might be due to invalid IL or missing references)
		//IL_042d: Unknown result type (might be due to invalid IL or missing references)
		//IL_04ad: Unknown result type (might be due to invalid IL or missing references)
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		Core.Attitude.RCS_auto = GUILayout.Toggle(Core.Attitude.RCS_auto, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox1"), Array.Empty<GUILayoutOption>());
		int num = Core.Attitude.activeController;
		if (GUILayout.Toggle(num == 0, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox2"), Array.Empty<GUILayoutOption>()))
		{
			num = 0;
		}
		if (GUILayout.Toggle(num == 1, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox3"), Array.Empty<GUILayoutOption>()))
		{
			num = 1;
		}
		if (GUILayout.Toggle(num == 2, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox4"), Array.Empty<GUILayoutOption>()))
		{
			num = 2;
		}
		if (GUILayout.Toggle(num == 3, "BetterController", Array.Empty<GUILayoutOption>()))
		{
			num = 3;
		}
		if (GUILayout.Toggle(num == 4, "LQRController", Array.Empty<GUILayoutOption>()))
		{
			num = 4;
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Space(20f);
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		Core.Attitude.Controller.GUI();
		GUILayout.EndVertical();
		GUILayout.EndHorizontal();
		if (num != Core.Attitude.activeController)
		{
			Core.Attitude.SetActiveController(num);
		}
		showInfos = GUILayout.Toggle(showInfos, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox5"), Array.Empty<GUILayoutOption>());
		if (showInfos)
		{
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeAdjust_Label1"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label(MuUtils.PrettyPrint(Core.Attitude.AxisControl, "F0"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeAdjust_Label2"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label("|" + ((Vector3d)(ref Core.Attitude.torque)).magnitude.ToString("F3") + "| " + MuUtils.PrettyPrint(Core.Attitude.torque), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeAdjust_Label3"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label("|" + ((Vector3d)(ref base.VesselState.TorqueReactionSpeed)).magnitude.ToString("F3") + "| " + MuUtils.PrettyPrint(base.VesselState.TorqueReactionSpeed), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.EndHorizontal();
			Vector3d vector = Vector3d.Scale(base.VesselState.MoI, Core.Attitude.torque.InvertNoNaN());
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeAdjust_Label4"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label("|" + ((Vector3d)(ref vector)).magnitude.ToString("F3") + "| " + MuUtils.PrettyPrint(vector), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeAdjust_Label5"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label("|" + ((Vector3d)(ref base.VesselState.MoI)).magnitude.ToString("F3") + "| " + MuUtils.PrettyPrint(base.VesselState.MoI), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeAdjust_Label6"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label("|" + ((Vector3)(ref base.Vessel.angularVelocity)).magnitude.ToString("F3") + "| " + MuUtils.PrettyPrint(Vector3d.op_Implicit(base.Vessel.angularVelocity)), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeAdjust_Label7"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label("|" + ((Vector3d)(ref base.VesselState.AngularMomentum)).magnitude.ToString("F3") + "| " + MuUtils.PrettyPrint(base.VesselState.AngularMomentum), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.EndHorizontal();
			GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
			GUILayout.Label(Localizer.Format("#MechJeb_AttitudeAdjust_Label8"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
			GUILayout.Label(TimeWarp.fixedDeltaTime.ToString("F3"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
			GUILayout.EndHorizontal();
		}
		MechJebModuleDebugArrows computerModule = Core.GetComputerModule<MechJebModuleDebugArrows>();
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_AttitudeAdjust_Label9"), computerModule.arrowsLength, "", 50f);
		computerModule.seeThrough = GUILayout.Toggle(computerModule.seeThrough, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox6"), Array.Empty<GUILayoutOption>());
		computerModule.comSphereActive = GUILayout.Toggle(computerModule.comSphereActive, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox7"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		computerModule.colSphereActive = GUILayout.Toggle(computerModule.colSphereActive, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox8"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		computerModule.cotSphereActive = GUILayout.Toggle(computerModule.cotSphereActive, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox9"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_AttitudeAdjust_Label10"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		computerModule.comSphereRadius.Text = GUILayout.TextField(computerModule.comSphereRadius.Text, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(40f) });
		GUILayout.EndHorizontal();
		computerModule.displayAtCoM = GUILayout.Toggle(computerModule.displayAtCoM, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox10"), Array.Empty<GUILayoutOption>());
		computerModule.srfVelocityArrowActive = GUILayout.Toggle(computerModule.srfVelocityArrowActive, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox11"), Array.Empty<GUILayoutOption>());
		computerModule.obtVelocityArrowActive = GUILayout.Toggle(computerModule.obtVelocityArrowActive, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox12"), Array.Empty<GUILayoutOption>());
		computerModule.dotArrowActive = GUILayout.Toggle(computerModule.dotArrowActive, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox13"), Array.Empty<GUILayoutOption>());
		computerModule.forwardArrowActive = GUILayout.Toggle(computerModule.forwardArrowActive, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox14"), Array.Empty<GUILayoutOption>());
		computerModule.requestedAttitudeArrowActive = GUILayout.Toggle(computerModule.requestedAttitudeArrowActive, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox15"), Array.Empty<GUILayoutOption>());
		computerModule.debugArrowActive = GUILayout.Toggle(computerModule.debugArrowActive, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox16"), Array.Empty<GUILayoutOption>());
		computerModule.debugArrow2Active = GUILayout.Toggle(computerModule.debugArrow2Active, Localizer.Format("#MechJeb_AttitudeAdjust_checkbox17"), Array.Empty<GUILayoutOption>());
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(350f),
			GUILayout.Height(150f)
		};
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_AttitudeAdjust_title");
	}

	public override string IconName()
	{
		return "Attitude Adjustment";
	}
}
