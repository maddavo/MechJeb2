using System;
using System.Reflection;

namespace MuMech;

public class GeneralInfoItem : InfoItem
{
	private readonly Action draw;

	private readonly object obj;

	public GeneralInfoItem(object obj, MethodInfo method, GeneralInfoItemAttribute attribute)
		: base(attribute)
	{
		id = GetType().Name.Replace("InfoItem", "") + ":" + obj.GetType().Name.Replace("MechJebModule", "") + "." + method.Name;
		draw = (Action)Delegate.CreateDelegate(typeof(Action), obj, method);
		this.obj = obj;
	}

	public override void DrawItem()
	{
		draw();
	}

	public override void UpdateItem()
	{
		if (obj is MechJebModuleInfoItems mechJebModuleInfoItems)
		{
			mechJebModuleInfoItems.UpdateItems();
		}
	}
}
