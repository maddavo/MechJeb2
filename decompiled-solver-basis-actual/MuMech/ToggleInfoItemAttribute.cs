using System;

namespace MuMech;

[AttributeUsage(AttributeTargets.Property | AttributeTargets.Field)]
public class ToggleInfoItemAttribute : InfoItemAttribute
{
	public ToggleInfoItemAttribute(string name, InfoItem.Category category)
		: base(name, category)
	{
	}
}
