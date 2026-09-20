using System;

namespace MuMech;

[AttributeUsage(AttributeTargets.Method)]
public class ActionInfoItemAttribute : InfoItemAttribute
{
	public ActionInfoItemAttribute(string name, InfoItem.Category category)
		: base(name, category)
	{
	}
}
