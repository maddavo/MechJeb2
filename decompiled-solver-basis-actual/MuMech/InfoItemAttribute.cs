using System;

namespace MuMech;

public abstract class InfoItemAttribute : Attribute
{
	public readonly string name;

	public InfoItem.Category category;

	public string description = "";

	public string tooltip = "";

	public bool showInEditor;

	public bool showInFlight = true;

	public InfoItemAttribute(string name, InfoItem.Category category)
	{
		this.name = name;
		this.category = category;
		description = name;
	}
}
