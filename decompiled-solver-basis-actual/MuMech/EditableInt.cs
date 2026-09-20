using System.Text.RegularExpressions;

namespace MuMech;

public class EditableInt : IEditable
{
	[Persistent]
	public int ValConfig;

	private bool _parsed;

	[Persistent]
	public string TextConfig;

	public int Val
	{
		get
		{
			return ValConfig;
		}
		set
		{
			ValConfig = value;
			TextConfig = value.ToString();
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
			TextConfig = Regex.Replace(TextConfig, "[^\\d+-]", "");
			_parsed = int.TryParse(TextConfig, out var result);
			if (_parsed)
			{
				Val = result;
			}
		}
	}

	public EditableInt(int val)
	{
		Val = val;
		TextConfig = val.ToString();
	}

	public static implicit operator int(EditableInt x)
	{
		return x.Val;
	}

	public static implicit operator EditableInt(int x)
	{
		return new EditableInt(x);
	}
}
