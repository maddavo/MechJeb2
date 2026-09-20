namespace MuMech.AttitudeControllers;

public abstract class BaseAttitudeController
{
	protected readonly MechJebModuleAttitudeController Ac;

	protected BaseAttitudeController(MechJebModuleAttitudeController controller)
	{
		Ac = controller;
	}

	public virtual void OnModuleDisabled()
	{
	}

	public virtual void OnModuleEnabled()
	{
	}

	public virtual void OnStart()
	{
	}

	public virtual void OnLoad(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		if (global != null && global.HasNode(GetType().Name))
		{
			ConfigNode.LoadObjectFromConfig((object)this, global.GetNode(GetType().Name), 4);
		}
		if (type != null && type.HasNode(GetType().Name))
		{
			ConfigNode.LoadObjectFromConfig((object)this, type.GetNode(GetType().Name), 2);
		}
		if (local != null && local.HasNode(GetType().Name))
		{
			ConfigNode.LoadObjectFromConfig((object)this, local.GetNode(GetType().Name), 1);
		}
	}

	public virtual void OnSave(ConfigNode local, ConfigNode type, ConfigNode global)
	{
		if (global != null)
		{
			ConfigNode.CreateConfigFromObject((object)this, 4, (ConfigNode)null).CopyTo(global.AddNode(GetType().Name));
		}
		if (type != null)
		{
			ConfigNode.CreateConfigFromObject((object)this, 2, (ConfigNode)null).CopyTo(type.AddNode(GetType().Name));
		}
		if (local != null)
		{
			ConfigNode.CreateConfigFromObject((object)this, 1, (ConfigNode)null).CopyTo(local.AddNode(GetType().Name));
		}
	}

	public virtual void ResetConfig()
	{
	}

	public virtual void OnFixedUpdate()
	{
	}

	public virtual void OnUpdate()
	{
	}

	public virtual void Reset()
	{
	}

	public abstract void DrivePre(FlightCtrlState s, out Vector3d act, out Vector3d deltaEuler);

	public virtual void GUI()
	{
	}

	public virtual void Reset(int i)
	{
		Reset();
	}
}
