using System.Text.RegularExpressions;

namespace MuMech;

public class EditableTime : EditableDouble
{
	public override double Val
	{
		get
		{
			return ValConfig;
		}
		set
		{
			ValConfig = value;
			TextConfig = GuiUtils.TimeToDHMS(ValConfig);
		}
	}

	public override string Text
	{
		get
		{
			return TextConfig;
		}
		set
		{
			TextConfig = value;
			TextConfig = Regex.Replace(TextConfig, "[^\\d+-.ydhms ,]", "");
			Parsed = double.TryParse(TextConfig, out var result);
			if (Parsed)
			{
				ValConfig = result;
				return;
			}
			Parsed = GuiUtils.TryParseDHMS(TextConfig, out result);
			if (Parsed)
			{
				ValConfig = result;
			}
		}
	}

	public EditableTime()
		: this(0.0)
	{
	}

	public EditableTime(double seconds)
		: base(seconds)
	{
		TextConfig = GuiUtils.TimeToDHMS(seconds);
	}

	public static implicit operator EditableTime(double x)
	{
		return new EditableTime(x);
	}

	public override string ToString()
	{
		return GuiUtils.TimeToDHMS(Val);
	}
}
