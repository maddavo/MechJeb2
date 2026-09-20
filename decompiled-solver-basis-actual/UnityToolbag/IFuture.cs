using System;

namespace UnityToolbag;

public interface IFuture<T>
{
	FutureState state { get; }

	T value { get; }

	Exception error { get; }

	IFuture<T> OnSuccess(FutureCallback<T> callback);

	IFuture<T> OnError(FutureCallback<T> callback);

	IFuture<T> OnComplete(FutureCallback<T> callback);
}
