namespace MuMech;

public class EditableDouble : EditableDoubleMult
{
	public EditableDouble(double val)
		: base(val)
	{
	}

	public static implicit operator EditableDouble(double x)
	{
		return new EditableDouble(x);
	}
}
