using KSP.Localization;

namespace MuMech;

public class InfoItem
{
	public enum Category
	{
		Orbit,
		Surface,
		Vessel,
		Target,
		Recorder,
		Thrust,
		Rover,
		Misc,
		Hoverslam
	}

	public readonly string name;

	public readonly string localizedName;

	public readonly string localizedTooltip;

	public readonly string description;

	public readonly bool showInEditor;

	public readonly bool showInFlight;

	public readonly Category category;

	[Persistent]
	public string id;

	public InfoItem()
	{
	}

	public InfoItem(InfoItemAttribute attribute)
	{
		name = attribute.name;
		localizedName = Localizer.Format(name);
		localizedTooltip = (string.IsNullOrEmpty(attribute.tooltip) ? "" : Localizer.Format(attribute.tooltip));
		category = attribute.category;
		description = attribute.description;
		showInEditor = attribute.showInEditor;
		showInFlight = attribute.showInFlight;
	}

	public virtual void DrawItem()
	{
	}

	public virtual void UpdateItem()
	{
	}
}
