using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public class MechJebModuleRCSBalancerWindow : DisplayModule
{
	public MechJebModuleRCSBalancer balancer;

	public override void OnStart(StartState state)
	{
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		balancer = Core.GetComputerModule<MechJebModuleRCSBalancer>();
		if (balancer.smartTranslation)
		{
			balancer.Users.Add(this);
		}
		base.OnStart(state);
	}

	private void SimpleTextInfo(string left, string right)
	{
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(left, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutExpandWidth });
		GUILayout.Label(right, (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutNoExpandWidth });
		GUILayout.EndHorizontal();
	}

	protected override void WindowGUI(int windowID)
	{
		GUILayout.BeginVertical(Array.Empty<GUILayoutOption>());
		bool smartTranslation = balancer.smartTranslation;
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		balancer.smartTranslation = GUILayout.Toggle(balancer.smartTranslation, Localizer.Format("#MechJeb_RCSBalancer_checkbox1"), (GUILayoutOption[])(object)new GUILayoutOption[1] { GuiUtils.LayoutWidth(130f) });
		GUILayout.EndHorizontal();
		if (smartTranslation != balancer.smartTranslation)
		{
			balancer.ResetThrusterForces();
			if (balancer.smartTranslation)
			{
				balancer.Users.Add(this);
			}
			else
			{
				balancer.Users.Remove(this);
			}
		}
		if (balancer.smartTranslation)
		{
			double num = balancer.overdrive;
			double num2 = balancer.overdriveScale;
			double num3 = balancer.tuningParamFactorTorque;
			double num4 = balancer.tuningParamFactorTranslate;
			double num5 = balancer.tuningParamFactorWaste;
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label1"), balancer.overdrive, "%");
			double num6 = GUILayout.HorizontalSlider((float)(double)balancer.overdrive, 0f, 1f, Array.Empty<GUILayoutOption>());
			if (Math.Round(Math.Abs(num6 - num), 3) > 0.0)
			{
				double val = Math.Round(num6, 3);
				balancer.overdrive = new EditableDoubleMult(val, 0.01);
			}
			GUILayout.Label(Localizer.Format("#MechJeb_RCSBalancer_label2"), Array.Empty<GUILayoutOption>());
			balancer.advancedOptions = GUILayout.Toggle(balancer.advancedOptions, Localizer.Format("#MechJeb_RCSBalancer_checkbox2"), Array.Empty<GUILayoutOption>());
			if (balancer.advancedOptions)
			{
				GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label3"), balancer.overdriveScale);
				GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label4"), balancer.tuningParamFactorTorque);
				GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label5"), balancer.tuningParamFactorTranslate);
				GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label6"), balancer.tuningParamFactorWaste);
			}
			if (num != (double)balancer.overdrive || num2 != (double)balancer.overdriveScale || num3 != (double)balancer.tuningParamFactorTorque || num4 != (double)balancer.tuningParamFactorTranslate || num5 != (double)balancer.tuningParamFactorWaste)
			{
				balancer.UpdateTuningParameters();
			}
		}
		if (balancer.smartTranslation)
		{
			balancer.Users.Add(this);
		}
		else
		{
			balancer.Users.Remove(this);
		}
		GUILayout.EndVertical();
		base.WindowGUI(windowID);
	}

	protected override GUILayoutOption[] WindowOptions()
	{
		return (GUILayoutOption[])(object)new GUILayoutOption[2]
		{
			GuiUtils.LayoutWidth(240f),
			GUILayout.Height(30f)
		};
	}

	public override string GetName()
	{
		return Localizer.Format("#MechJeb_RCSBalancer_title");
	}

	public override string IconName()
	{
		return "RCS Balancer";
	}

	public MechJebModuleRCSBalancerWindow(MechJebCore core)
		: base(core)
	{
	}
}
