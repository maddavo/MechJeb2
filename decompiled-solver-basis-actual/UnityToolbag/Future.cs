using System;
using System.Collections.Generic;
using System.Threading;

namespace UnityToolbag;

public sealed class Future<T> : IFuture<T>
{
	private volatile FutureState _state;

	private T _value;

	private Exception _error;

	private readonly List<FutureCallback<T>> _successCallbacks = new List<FutureCallback<T>>();

	private readonly List<FutureCallback<T>> _errorCallbacks = new List<FutureCallback<T>>();

	public FutureState state => _state;

	public T value
	{
		get
		{
			if (_state != FutureState.Success)
			{
				throw new InvalidOperationException("value is not available unless state is Success.");
			}
			return _value;
		}
	}

	public Exception error
	{
		get
		{
			if (_state != FutureState.Error)
			{
				throw new InvalidOperationException("error is not available unless state is Error.");
			}
			return _error;
		}
	}

	public Future()
	{
		_state = FutureState.Pending;
	}

	public IFuture<T> OnSuccess(FutureCallback<T> callback)
	{
		if (_state == FutureState.Success)
		{
			if (Dispatcher.isMainThread)
			{
				callback(this);
			}
			else
			{
				Dispatcher.InvokeAsync(delegate
				{
					callback(this);
				});
			}
		}
		else if (_state != FutureState.Error && !_successCallbacks.Contains(callback))
		{
			_successCallbacks.Add(callback);
		}
		return this;
	}

	public IFuture<T> OnError(FutureCallback<T> callback)
	{
		if (_state == FutureState.Error)
		{
			if (Dispatcher.isMainThread)
			{
				callback(this);
			}
			else
			{
				Dispatcher.InvokeAsync(delegate
				{
					callback(this);
				});
			}
		}
		else if (_state != FutureState.Success && !_errorCallbacks.Contains(callback))
		{
			_errorCallbacks.Add(callback);
		}
		return this;
	}

	public IFuture<T> OnComplete(FutureCallback<T> callback)
	{
		if (_state == FutureState.Success || _state == FutureState.Error)
		{
			if (Dispatcher.isMainThread)
			{
				callback(this);
			}
			else
			{
				Dispatcher.InvokeAsync(delegate
				{
					callback(this);
				});
			}
		}
		else
		{
			if (!_successCallbacks.Contains(callback))
			{
				_successCallbacks.Add(callback);
			}
			if (!_errorCallbacks.Contains(callback))
			{
				_errorCallbacks.Add(callback);
			}
		}
		return this;
	}

	public IFuture<T> Process(Func<T> func)
	{
		if (_state != 0)
		{
			throw new InvalidOperationException("Cannot process a future that isn't in the Pending state.");
		}
		_state = FutureState.Processing;
		ThreadPool.QueueUserWorkItem(delegate
		{
			try
			{
				AssignImpl(func());
			}
			catch (Exception ex)
			{
				FailImpl(ex);
			}
		});
		return this;
	}

	public void Assign(T value)
	{
		if (_state != 0)
		{
			throw new InvalidOperationException("Cannot assign a value to a future that isn't in the Pending state.");
		}
		AssignImpl(value);
	}

	public void Fail(Exception error)
	{
		if (_state != 0)
		{
			throw new InvalidOperationException("Cannot fail future that isn't in the Pending state.");
		}
		FailImpl(error);
	}

	private void AssignImpl(T value)
	{
		_value = value;
		_error = null;
		_state = FutureState.Success;
		Dispatcher.InvokeAsync(FlushSuccessCallbacks);
	}

	private void FailImpl(Exception error)
	{
		_value = default(T);
		_error = error;
		_state = FutureState.Error;
		Dispatcher.InvokeAsync(FlushErrorCallbacks);
	}

	private void FlushSuccessCallbacks()
	{
		foreach (FutureCallback<T> successCallback in _successCallbacks)
		{
			successCallback(this);
		}
		_successCallbacks.Clear();
		_errorCallbacks.Clear();
	}

	private void FlushErrorCallbacks()
	{
		foreach (FutureCallback<T> errorCallback in _errorCallbacks)
		{
			errorCallback(this);
		}
		_successCallbacks.Clear();
		_errorCallbacks.Clear();
	}
}
