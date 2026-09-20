using System.Collections.Generic;

namespace MuMech;

public class ModuleEvent
{
	public delegate void OnEvent();

	private readonly List<OnEvent> _events = new List<OnEvent>();

	private readonly Dictionary<OnEvent, int> _eventIndex = new Dictionary<OnEvent, int>();

	public void Add(OnEvent evt)
	{
		if (!_eventIndex.ContainsKey(evt))
		{
			_events.Add(evt);
			_eventIndex.Add(evt, _events.Count - 1);
		}
	}

	public void Remove(OnEvent evt)
	{
		if (_eventIndex.ContainsKey(evt))
		{
			_events.RemoveAt(_eventIndex[evt]);
			_eventIndex.Remove(evt);
		}
	}

	public void Clear()
	{
		_events.Clear();
		_eventIndex.Clear();
	}

	public void Fire(bool reverse)
	{
		if (reverse)
		{
			for (int num = _events.Count - 1; num >= 0; num--)
			{
				_events[num]();
			}
		}
		else
		{
			for (int i = 0; i < _events.Count; i++)
			{
				_events[i]();
			}
		}
	}
}
