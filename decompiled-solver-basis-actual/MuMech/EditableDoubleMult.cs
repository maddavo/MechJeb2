using System.Text.RegularExpressions;

namespace MuMech;

public class EditableDoubleMult : IEditable
{
	[Persistent]
	public double ValConfig;

	private readonly double _multiplier;

	protected bool Parsed;

	[Persistent]
	public string TextConfig;

	public virtual double Val
	{
		get
		{
			return ValConfig;
		}
		set
		{
			ValConfig = value;
			TextConfig = (ValConfig / _multiplier).ToString();
		}
	}

	public virtual string Text
	{
		get
		{
			return TextConfig;
		}
		set
		{
			TextConfig = value;
			TextConfig = Regex.Replace(TextConfig, "[^\\d+-.]", "");
			Parsed = double.TryParse(TextConfig, out var result);
			if (Parsed)
			{
				ValConfig = result * _multiplier;
			}
		}
	}

	public EditableDoubleMult()
		: this(0.0)
	{
	}

	public EditableDoubleMult(double val, double multiplier = 1.0)
	{
		Val = val;
		_multiplier = multiplier;
		TextConfig = (val / multiplier).ToString();
	}

	public static implicit operator double(EditableDoubleMult x)
	{
		return x.Val;
	}

	public override string ToString()
	{
		return Val.ToString();
	}
}
