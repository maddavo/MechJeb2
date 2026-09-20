using System;
using System.Collections.Generic;
using System.Reflection;
using KSP.Localization;
using UnityEngine;

namespace MuMech;

public abstract class Operation
{
	protected string? ErrorMessage = "";

	private static readonly List<Type> _operations = new List<Type>();

	public virtual bool Draggable => true;

	public string? GetErrorMessage()
	{
		return ErrorMessage;
	}

	public abstract string GetName();

	public abstract void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target);

	protected abstract List<ManeuverParameters>? MakeNodesImpl(Orbit o, double universalTime, MechJebModuleTargetController target);

	public List<ManeuverParameters>? MakeNodes(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		ErrorMessage = "";
		try
		{
			return MakeNodesImpl(o, universalTime, target);
		}
		catch (OperationException ex)
		{
			ErrorMessage = ex.Message;
			return null;
		}
		catch (Exception ex2)
		{
			Debug.LogException(ex2);
			ErrorMessage = Localizer.Format("#MechJeb_Maneu_errorMessage");
			return null;
		}
	}

	private static void AddTypes(Type[] types)
	{
		foreach (Type type in types)
		{
			if ((object)type != null && !type.IsAbstract && typeof(Operation).IsAssignableFrom(type) && type.GetConstructor(Type.EmptyTypes) != null)
			{
				_operations.Add(type);
			}
		}
	}

	public static Operation[] GetAvailableOperations()
	{
		if (_operations.Count == 0)
		{
			Assembly[] assemblies = AppDomain.CurrentDomain.GetAssemblies();
			foreach (Assembly assembly in assemblies)
			{
				try
				{
					AddTypes(assembly.GetTypes());
				}
				catch (ReflectionTypeLoadException ex)
				{
					AddTypes(ex.Types);
				}
				catch (InvalidOperationException)
				{
				}
			}
			Debug.Log((object)("ManeuverPlanner initialization: found " + _operations.Count + " maneuvers"));
		}
		List<Operation> list = _operations.ConvertAll((Type t) => (Operation)t.GetConstructor(Type.EmptyTypes).Invoke(null));
		list.Sort((Operation x, Operation y) => string.Compare(x.GetName(), y.GetName(), StringComparison.Ordinal));
		return list.ToArray();
	}
}
