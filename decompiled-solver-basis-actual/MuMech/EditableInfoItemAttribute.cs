using System;

namespace MuMech;

[AttributeUsage(AttributeTargets.Property | AttributeTargets.Field)]
public class EditableInfoItemAttribute : InfoItemAttribute
{
	public string rightLabel = "";

	public float width = 100f;

	public bool expandWidth;

	public EditableInfoItemAttribute(string name, InfoItem.Category category)
		: base(name, category)
	{
	}
}
