using System;
using System.Collections.Generic;
using KSP.Localization;
using MechJebLib.Utils;
using UnityEngine;

namespace MuMech;

public class OperationAdvancedTransfer : Operation
{
	private enum Mode
	{
		LIMITED_TIME,
		PORKCHOP
	}

	private static readonly string _name = Localizer.Format("#MechJeb_AdvancedTransfer_title");

	private static readonly string[] _modeNames = new string[2]
	{
		Localizer.Format("#MechJeb_adv_modeName1"),
		Localizer.Format("#MechJeb_adv_modeName2")
	};

	private double _minDepartureTime;

	private double _minTransferTime;

	private double _maxDepartureTime;

	private double _maxTransferTime;

	public readonly EditableTime MaxArrivalTime = new EditableTime();

	private bool _includeCaptureBurn;

	private EditableDouble _periapsisHeight = new EditableDouble(0.0);

	private const double MIN_SAMPLING_STEP = 43200.0;

	private Mode _selectionMode = Mode.PORKCHOP;

	private int _windowWidth;

	private CelestialBody? _lastTargetCelestial;

	private TransferCalculator? _worker;

	private PlotArea? _plot;

	private static Texture2D? _texture;

	private bool _draggable = true;

	private const int PORKCHOP_HEIGHT = 200;

	private static GUIStyle? _progressStyle;

	private bool _layoutSkipped;

	public override bool Draggable => _draggable;

	public override string GetName()
	{
		return _name;
	}

	private string? CheckPreconditions(Orbit o, MechJebModuleTargetController target)
	{
		if (o.eccentricity >= 1.0)
		{
			return Localizer.Format("#MechJeb_adv_Preconditions1");
		}
		if (o.ApR >= o.referenceBody.sphereOfInfluence)
		{
			return Localizer.Format("#MechJeb_adv_Preconditions2", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.displayName) });
		}
		if (!target.NormalTargetExists)
		{
			return Localizer.Format("#MechJeb_adv_Preconditions3");
		}
		if ((Object)(object)o.referenceBody.referenceBody == (Object)null)
		{
			return Localizer.Format("#MechJeb_adv_Preconditions4", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.displayName) });
		}
		if ((Object)(object)o.referenceBody.referenceBody != (Object)(object)target.TargetOrbit.referenceBody)
		{
			if ((Object)(object)o.referenceBody == (Object)(object)target.TargetOrbit.referenceBody)
			{
				return Localizer.Format("#MechJeb_adv_Preconditions5", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.displayName) });
			}
			return Localizer.Format("#MechJeb_adv_Preconditions6", new string[3]
			{
				LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.displayName),
				LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.displayName),
				LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.referenceBody.displayName)
			});
		}
		if ((Object)(object)o.referenceBody == (Object)(object)Planetarium.fetch.Sun)
		{
			return Localizer.Format("#MechJeb_adv_Preconditions7");
		}
		if (target.Target is CelestialBody && (Object)(object)o.referenceBody == (Object)(object)target.targetBody)
		{
			return Localizer.Format("#MechJeb_adv_Preconditions8", new string[1] { LingoonaGrammarExtensions.LocalizeRemoveGender(o.referenceBody.displayName) });
		}
		return null;
	}

	private void ComputeStuff(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		ErrorMessage = CheckPreconditions(o, target);
		if (ErrorMessage == null)
		{
			ErrorMessage = "";
			if (_worker != null)
			{
				_worker.Stop = true;
			}
			_plot = null;
			switch (_selectionMode)
			{
			case Mode.LIMITED_TIME:
				_worker = new TransferCalculator(o, target.TargetOrbit, universalTime, MaxArrivalTime, 43200.0, _includeCaptureBurn);
				break;
			case Mode.PORKCHOP:
				_worker = new AllGraphTransferCalculator(o, target.TargetOrbit, _minDepartureTime, _maxDepartureTime, _minTransferTime, _maxTransferTime, _windowWidth, 200, _includeCaptureBurn);
				break;
			}
		}
	}

	private void ComputeTimes(Orbit? o, Orbit? destination, double universalTime)
	{
		if (destination != null && ((o != null) ? o.referenceBody.orbit : null) != null)
		{
			double num = o.referenceBody.orbit.SynodicPeriod(destination);
			double transferTime = OrbitUtil.GetTransferTime(o.referenceBody.orbit, destination);
			if (double.IsInfinity(num))
			{
				num = o.referenceBody.orbit.period;
			}
			_minDepartureTime = universalTime;
			_minTransferTime = 3600.0;
			_maxDepartureTime = _minDepartureTime + num * 1.5;
			_maxTransferTime = transferTime * 2.0;
			MaxArrivalTime.Val = num * 1.5 + transferTime * 2.0;
		}
	}

	private void DoPorkchopGui(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		//IL_0048: Invalid comparison between Unknown and I4
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Invalid comparison between Unknown and I4
		//IL_02a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02cb: Unknown result type (might be due to invalid IL or missing references)
		//IL_02d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_02e0: Unknown result type (might be due to invalid IL or missing references)
		//IL_02f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_0304: Expected O, but got Unknown
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ae: Invalid comparison between Unknown and I4
		//IL_0119: Unknown result type (might be due to invalid IL or missing references)
		//IL_0123: Expected O, but got Unknown
		ITargetable target2 = target.Target;
		CelestialBody val = (CelestialBody)(object)((target2 is CelestialBody) ? target2 : null);
		if (_worker == null)
		{
			if ((int)Event.current.type == 8)
			{
				_layoutSkipped = true;
			}
			return;
		}
		if ((int)Event.current.type == 8)
		{
			_layoutSkipped = false;
		}
		if (_layoutSkipped)
		{
			return;
		}
		string text = " - ";
		string text2 = " - ";
		string text3 = " - ";
		if (_worker.Finished && _worker.Computed.GetLength(1) == 200 && _plot == null && (int)Event.current.type == 8)
		{
			int length = _worker.Computed.GetLength(0);
			int length2 = _worker.Computed.GetLength(1);
			if (_texture != null && (((Texture)_texture).width != length || ((Texture)_texture).height != length2))
			{
				Object.Destroy((Object)(object)_texture);
				_texture = null;
			}
			if (_texture == null)
			{
				_texture = new Texture2D(length, length2, (TextureFormat)3, false);
			}
			Porkchop.RefreshTexture(_worker.Computed, _texture);
			_plot = new PlotArea(_worker.MinDepartureTime, _worker.MaxDepartureTime, _worker.MinTransferTime, _worker.MaxTransferTime, _texture, delegate(double xMin, double xMax, double yMin, double yMax)
			{
				_minDepartureTime = Math.Max(xMin, universalTime);
				_maxDepartureTime = xMax;
				_minTransferTime = Math.Max(yMin, 3600.0);
				_maxTransferTime = yMax;
				GUI.changed = true;
			})
			{
				SelectedPoint = new int[2] { _worker.BestDate, _worker.BestDuration }
			};
		}
		if (_plot != null)
		{
			int[] array = _plot.SelectedPoint;
			if (_plot.HoveredPoint != null)
			{
				array = _plot.HoveredPoint;
			}
			double num = _worker.Computed[array[0], array[1]];
			if (num > 0.0)
			{
				text = Statics.ToSI(num, 4, int.MaxValue) + "m/s";
				text2 = ((_worker.DateFromIndex(array[0]) < Planetarium.GetUniversalTime()) ? Localizer.Format("#MechJeb_adv_label1") : GuiUtils.TimeToDHMS(_worker.DateFromIndex(array[0]) - Planetarium.GetUniversalTime()));
				text3 = GuiUtils.TimeToDHMS(_worker.DurationFromIndex(array[1]));
			}
			_plot.DoGUI();
			if (!_plot.Draggable)
			{
				_draggable = false;
			}
		}
		else
		{
			if (_progressStyle == null)
			{
				GUIStyle val2 = new GUIStyle
				{
					font = GuiUtils.Skin.font,
					fontSize = GuiUtils.Skin.label.fontSize,
					fontStyle = GuiUtils.Skin.label.fontStyle
				};
				val2.normal.textColor = GuiUtils.Skin.label.normal.textColor;
				_progressStyle = val2;
			}
			GUILayout.Box(Localizer.Format("#MechJeb_adv_computing") + _worker.Progress + "%", _progressStyle, (GUILayoutOption[])(object)new GUILayoutOption[2]
			{
				GuiUtils.LayoutWidth(_windowWidth),
				GUILayout.Height(200f)
			});
		}
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label("ΔV: " + text, Array.Empty<GUILayoutOption>());
		GUILayout.FlexibleSpace();
		if (GUILayout.Button(Localizer.Format("#MechJeb_adv_reset_button"), GuiUtils.YellowOnHover, Array.Empty<GUILayoutOption>()))
		{
			ComputeTimes(o, target.TargetOrbit, universalTime);
		}
		GUILayout.EndHorizontal();
		_includeCaptureBurn = GUILayout.Toggle(_includeCaptureBurn, Localizer.Format("#MechJeb_adv_captureburn"), Array.Empty<GUILayoutOption>());
		if ((Object)(object)val != (Object)null && (Object)(object)_lastTargetCelestial != (Object)(object)val)
		{
			if (val.atmosphere)
			{
				_periapsisHeight = val.atmosphereDepth / 1000.0 + 10.0;
			}
			else
			{
				_periapsisHeight = 100.0;
			}
		}
		GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_adv_periapsis"), _periapsisHeight, "km");
		GUILayout.BeginHorizontal(Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_adv_label2"), Array.Empty<GUILayoutOption>());
		GUILayout.FlexibleSpace();
		if (GUILayout.Button(Localizer.Format("#MechJeb_adv_button1"), Array.Empty<GUILayoutOption>()) && _plot != null)
		{
			_plot.SelectedPoint = new int[2] { _worker.BestDate, _worker.BestDuration };
			GUI.changed = false;
		}
		if (GUILayout.Button(Localizer.Format("#MechJeb_adv_button2"), Array.Empty<GUILayoutOption>()) && _plot != null)
		{
			int num2 = 0;
			for (int i = 1; i < _worker.Computed.GetLength(1); i++)
			{
				if (_worker.Computed[0, num2] > _worker.Computed[0, i])
				{
					num2 = i;
				}
			}
			_plot.SelectedPoint = new int[2] { 0, num2 };
			GUI.changed = false;
		}
		GUILayout.EndHorizontal();
		GUILayout.Label(Localizer.Format("#MechJeb_adv_label3") + " " + text2, Array.Empty<GUILayoutOption>());
		GUILayout.Label(Localizer.Format("#MechJeb_adv_label4") + " " + text3, Array.Empty<GUILayoutOption>());
		_lastTargetCelestial = val;
	}

	public override void DoParametersGUI(Orbit o, double universalTime, MechJebModuleTargetController target)
	{
		//IL_005b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Invalid comparison between Unknown and I4
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_0069: Unknown result type (might be due to invalid IL or missing references)
		//IL_001c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Invalid comparison between Unknown and I4
		_draggable = true;
		if (_worker != null && !target.NormalTargetExists && (int)Event.current.type == 8)
		{
			_worker.Stop = true;
			_worker = null;
			_plot = null;
		}
		_selectionMode = (Mode)GuiUtils.ComboBox.Box((int)_selectionMode, _modeNames, this);
		if ((int)Event.current.type == 7)
		{
			Rect lastRect = GUILayoutUtility.GetLastRect();
			_windowWidth = (int)((Rect)(ref lastRect)).width;
		}
		switch (_selectionMode)
		{
		case Mode.LIMITED_TIME:
		{
			GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_adv_label5"), MaxArrivalTime);
			TransferCalculator worker = _worker;
			if (worker != null && !worker.Finished)
			{
				GuiUtils.SimpleLabel(Localizer.Format("#MechJeb_adv_computing") + _worker.Progress + "%");
			}
			break;
		}
		case Mode.PORKCHOP:
			DoPorkchopGui(o, universalTime, target);
			break;
		}
		if (_worker == null || _worker.DestinationOrbit != target.TargetOrbit || _worker.OriginOrbit != o)
		{
			ComputeTimes(o, target.TargetOrbit, universalTime);
		}
		if (GUI.changed || _worker == null || _worker.DestinationOrbit != target.TargetOrbit || _worker.OriginOrbit != o)
		{
			ComputeStuff(o, universalTime, target);
		}
	}

	private (double epoch, double arrivalDt, double arrivalDtLower, double arrivalDtUpper) ResolveTimes(TransferCalculator w)
	{
		PlotArea? plot = _plot;
		int? obj;
		if (plot == null)
		{
			obj = null;
		}
		else
		{
			int[] selectedPoint = plot.SelectedPoint;
			obj = ((selectedPoint != null) ? new int?(selectedPoint[0]) : null);
		}
		int num = obj ?? w.BestDate;
		PlotArea? plot2 = _plot;
		int? obj2;
		if (plot2 == null)
		{
			obj2 = null;
		}
		else
		{
			int[] selectedPoint2 = plot2.SelectedPoint;
			obj2 = ((selectedPoint2 != null) ? new int?(selectedPoint2[1]) : null);
		}
		int num2 = obj2 ?? w.BestDuration;
		double item = w.DateFromIndex(num);
		double num3 = w.DurationFromIndex(num2);
		double num4 = (w.MaxDepartureTime - w.MinDepartureTime) / (double)w.DateSamples;
		double item2 = num3 - 0.5 * num4;
		double item3 = num3 + 0.5 * num4;
		if (num == w.BestDate && num2 == w.BestDuration)
		{
			item2 = 0.0;
			item3 = double.PositiveInfinity;
		}
		if (_selectionMode == Mode.LIMITED_TIME && (double)MaxArrivalTime > 0.0)
		{
			item3 = MaxArrivalTime;
		}
		return (epoch: item, arrivalDt: num3, arrivalDtLower: item2, arrivalDtUpper: item3);
	}

	protected override List<ManeuverParameters> MakeNodesImpl(Orbit o, double ut, MechJebModuleTargetController target)
	{
		string text = CheckPreconditions(o, target);
		if (text != null)
		{
			throw new OperationException(text);
		}
		TransferCalculator worker = _worker;
		if (worker != null)
		{
			if (!worker.Finished)
			{
				throw new OperationException(Localizer.Format("#MechJeb_adv_Exception1"));
			}
			if (_worker.ArrivalDate < 0.0)
			{
				throw new OperationException(Localizer.Format("#MechJeb_adv_Exception3"));
			}
			double targetPeR = (_lastTargetCelestial?.Radius ?? 0.0) + (double)_periapsisHeight * 1000.0;
			if (_selectionMode == Mode.PORKCHOP && _plot?.SelectedPoint == null)
			{
				throw new OperationException(Localizer.Format("#MechJeb_adv_Exception4"));
			}
			var (epoch, arrivalDt, arrivalDtLower, arrivalDtUpper) = ResolveTimes(_worker);
			return OrbitalManeuverCalculator.OptimizeEjectionToTarget(o, target, targetPeR, epoch, arrivalDt, arrivalDtLower, arrivalDtUpper);
		}
		ComputeStuff(o, ut, target);
		throw new OperationException(Localizer.Format("#MechJeb_adv_Exception2"));
	}
}
