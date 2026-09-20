using UnityEngine;

namespace MuMech;

public class MechJebAR202 : PartModule
{
	private enum LightColor
	{
		NEITHER,
		GREEN,
		RED
	}

	private MechJebCore core;

	private Light greenLight;

	private Renderer greenLightRenderer;

	private Transform greenLightTransform;

	private LightColor litLight;

	private Light redLight;

	private Renderer redLightRenderer;

	private Transform redLightTransform;

	private int emissionId;

	public override void OnStart(StartState state)
	{
		//IL_0017: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Invalid comparison between Unknown and I4
		core = ((PartModule)this).part.Modules.GetModule<MechJebCore>(0);
		if ((int)state != 0 && (int)state != 1)
		{
			InitializeLights();
		}
	}

	public void FixedUpdate()
	{
		if (!HighLogic.LoadedSceneIsEditor)
		{
			HandleLights();
		}
	}

	private void InitializeLights()
	{
		//IL_0103: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ca: Unknown result type (might be due to invalid IL or missing references)
		greenLightTransform = null;
		redLightTransform = null;
		Transform[] componentsInChildren = ((Component)this).GetComponentsInChildren<Transform>();
		foreach (Transform val in componentsInChildren)
		{
			if (((Object)val).name.Equals("light_green"))
			{
				greenLightTransform = val;
			}
			if (((Object)val).name.Equals("light_red"))
			{
				redLightTransform = val;
			}
		}
		emissionId = Shader.PropertyToID("_EmissiveColor");
		if ((Object)(object)greenLightTransform != (Object)null)
		{
			if ((Object)(object)((Component)greenLightTransform).GetComponent<Light>() == (Object)null)
			{
				greenLightRenderer = ((Component)greenLightTransform).GetComponent<Renderer>();
				greenLight = ((Component)greenLightTransform).gameObject.AddComponent<Light>();
				((Component)greenLight).transform.parent = greenLightTransform;
				greenLight.type = (LightType)2;
				greenLight.renderMode = (LightRenderMode)1;
				greenLight.shadows = (LightShadows)0;
				((Behaviour)greenLight).enabled = false;
				greenLight.color = Color.green;
				greenLight.range = 1.5f;
			}
			else
			{
				greenLight = ((Component)greenLightTransform).GetComponent<Light>();
			}
		}
		if ((Object)(object)redLightTransform != (Object)null)
		{
			if ((Object)(object)((Component)redLightTransform).GetComponent<Light>() == (Object)null)
			{
				redLightRenderer = ((Component)redLightTransform).GetComponent<Renderer>();
				redLight = ((Component)redLightTransform).gameObject.AddComponent<Light>();
				((Component)redLight).transform.parent = redLightTransform;
				redLight.type = (LightType)2;
				redLight.renderMode = (LightRenderMode)1;
				redLight.shadows = (LightShadows)0;
				((Behaviour)redLight).enabled = false;
				redLight.color = Color.red;
				redLight.range = 1.5f;
			}
			else
			{
				redLight = ((Component)redLightTransform).GetComponent<Light>();
			}
		}
	}

	private void HandleLights()
	{
		if ((Object)(object)greenLight == (Object)null || (Object)(object)redLight == (Object)null)
		{
			InitializeLights();
		}
		if ((Object)(object)greenLight == (Object)null || (Object)(object)redLight == (Object)null)
		{
			return;
		}
		if ((Object)(object)core == (Object)null || MapView.MapIsEnabled)
		{
			litLight = LightColor.NEITHER;
		}
		else
		{
			bool flag = false;
			if ((Object)(object)((PartModule)this).vessel.GetMasterMechJeb() == (Object)(object)core)
			{
				foreach (DisplayModule displayModule in core.GetDisplayModules(MechJebModuleMenu.DisplayOrder.instance))
				{
					if (!(displayModule is MechJebModuleMenu) && displayModule.Enabled && displayModule.ShowInCurrentScene)
					{
						flag = true;
					}
				}
			}
			litLight = (flag ? LightColor.GREEN : LightColor.RED);
		}
		switch (litLight)
		{
		case LightColor.GREEN:
			if (!((Behaviour)greenLight).enabled)
			{
				TurnOnLight(LightColor.GREEN);
			}
			if (((Behaviour)redLight).enabled)
			{
				TurnOffLight(LightColor.RED);
			}
			break;
		case LightColor.RED:
			if (((Behaviour)greenLight).enabled)
			{
				TurnOffLight(LightColor.GREEN);
			}
			if (!((Behaviour)redLight).enabled)
			{
				TurnOnLight(LightColor.RED);
			}
			break;
		case LightColor.NEITHER:
			if (((Behaviour)greenLight).enabled)
			{
				TurnOffLight(LightColor.GREEN);
			}
			if (((Behaviour)redLight).enabled)
			{
				TurnOffLight(LightColor.RED);
			}
			break;
		}
	}

	private void TurnOnLight(LightColor which)
	{
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		switch (which)
		{
		case LightColor.GREEN:
			if ((Object)(object)greenLightTransform != (Object)null)
			{
				greenLightRenderer.material.SetColor(emissionId, Color.green);
				((Behaviour)greenLight).enabled = true;
			}
			break;
		case LightColor.RED:
			if ((Object)(object)redLightTransform != (Object)null)
			{
				redLightRenderer.material.SetColor(emissionId, Color.red);
				((Behaviour)redLight).enabled = true;
			}
			break;
		}
	}

	private void TurnOffLight(LightColor which)
	{
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_005e: Unknown result type (might be due to invalid IL or missing references)
		switch (which)
		{
		case LightColor.GREEN:
			if ((Object)(object)greenLightTransform != (Object)null)
			{
				greenLightRenderer.material.SetColor(emissionId, Color.black);
				((Behaviour)greenLight).enabled = false;
			}
			break;
		case LightColor.RED:
			if ((Object)(object)redLightTransform != (Object)null)
			{
				redLightRenderer.material.SetColor(emissionId, Color.black);
				((Behaviour)redLight).enabled = false;
			}
			break;
		}
	}
}
