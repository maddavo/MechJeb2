using System.Collections.Generic;

namespace MuMech;

public class UserPool : List<object>
{
	private readonly ComputerModule _controlledModule;

	public UserPool(ComputerModule controlledModule)
	{
		_controlledModule = controlledModule;
	}

	public new void Add(object user)
	{
		if (user != null && !Contains(user))
		{
			base.Add(user);
		}
		_controlledModule.Enabled = true;
	}

	public new void Remove(object user)
	{
		if (user != null && Contains(user))
		{
			base.Remove(user);
		}
		if (base.Count == 0)
		{
			_controlledModule.Enabled = false;
		}
	}

	public new void Clear()
	{
		base.Clear();
		_controlledModule.Enabled = false;
	}

	public bool RecursiveUser(object user)
	{
		if (Contains(user))
		{
			return true;
		}
		using (Enumerator enumerator = GetEnumerator())
		{
			while (enumerator.MoveNext())
			{
				if (enumerator.Current is ComputerModule computerModule && computerModule != _controlledModule && computerModule.Users.RecursiveUser(user))
				{
					return true;
				}
			}
		}
		return false;
	}
}
