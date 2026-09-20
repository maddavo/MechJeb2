using System.Collections.Generic;
using System.Text.RegularExpressions;

namespace MuMech;

public class EditableIntList : IEditable
{
	[Persistent]
	public readonly List<int> Val = new List<int>();

	[Persistent]
	public string TextConfig = "";

	public string Text
	{
		get
		{
			return TextConfig;
		}
		set
		{
			TextConfig = value;
			TextConfig = Regex.Replace(TextConfig, "[^\\d-,]", "");
			Val.Clear();
			string[] array = TextConfig.Split(',');
			for (int i = 0; i < array.Length; i++)
			{
				string[] array2 = array[i].Split('-');
				if (int.TryParse(array2[0].Trim(), out var result) && int.TryParse(array2[array2.Length - 1].Trim(), out var result2))
				{
					for (int j = result; j <= result2; j++)
					{
						Val.Add(j);
					}
				}
			}
		}
	}
}
