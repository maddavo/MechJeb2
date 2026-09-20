using System;

namespace MuMech;

[AttributeUsage(AttributeTargets.Method)]
public class GeneralInfoItemAttribute : InfoItemAttribute
{
	public GeneralInfoItemAttribute(string name, InfoItem.Category category)
		: base(name, category)
	{
	}
}
