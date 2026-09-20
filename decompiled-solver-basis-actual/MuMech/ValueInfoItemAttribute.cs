using System;

namespace MuMech;

[AttributeUsage(AttributeTargets.Method | AttributeTargets.Property | AttributeTargets.Field)]
public class ValueInfoItemAttribute : InfoItemAttribute
{
	public string units = "";

	public string format = "";

	public int siSigFigs = 4;

	public readonly int siMaxPrecision = -33;

	public int timeDecimalPlaces;

	public float width = -1f;

	public ValueInfoItemAttribute(string name, InfoItem.Category category)
		: base(name, category)
	{
	}
}
