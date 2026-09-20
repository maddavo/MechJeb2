using System;
using System.Collections;
using System.Threading;
using UnityEngine;

namespace UnityToolbag;

[AddComponentMenu("UnityToolbag/Dispatcher")]
public class Dispatcher : MonoBehaviour
{
	private static Dispatcher _instance;

	private static bool _instanceExists;

	private static Thread _mainThread;

	private static readonly object _lockObject = new object();

	private static readonly Queue _actions = new Queue();

	public static bool isMainThread => Thread.CurrentThread == _mainThread;

	public static void InvokeAsync(Action action)
	{
		if (!_instanceExists)
		{
			Debug.LogError((object)"No Dispatcher exists in the scene. Actions will not be invoked!");
			return;
		}
		if (isMainThread)
		{
			action();
			return;
		}
		lock (_lockObject)
		{
			_actions.Enqueue(action);
		}
	}

	public static void Invoke(Action action)
	{
		if (!_instanceExists)
		{
			Debug.LogError((object)"No Dispatcher exists in the scene. Actions will not be invoked!");
			return;
		}
		bool hasRun = false;
		InvokeAsync(delegate
		{
			action();
			hasRun = true;
		});
		while (!hasRun)
		{
			Thread.Sleep(5);
		}
	}

	private void Awake()
	{
		if (Object.op_Implicit((Object)(object)_instance))
		{
			Object.DestroyImmediate((Object)(object)this);
			return;
		}
		_instance = this;
		_instanceExists = true;
		_mainThread = Thread.CurrentThread;
		Object.DontDestroyOnLoad((Object)(object)this);
	}

	private void OnDestroy()
	{
		if ((Object)(object)_instance == (Object)(object)this)
		{
			_instance = null;
			_instanceExists = false;
		}
	}

	private void Update()
	{
		lock (_lockObject)
		{
			while (_actions.Count > 0)
			{
				((Action)_actions.Dequeue())();
			}
		}
	}

	public static void CreateDispatcher()
	{
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		if (!_instanceExists)
		{
			Debug.Log((object)"[MechJeb2] Starting the Dispatcher");
			new GameObject(typeof(Dispatcher).Name).AddComponent<Dispatcher>();
		}
	}
}
