using System;
using KSP.Localization;

namespace MuMech.Landing;

public class CourseCorrection : AutopilotStep
{
	private const double MaxCorrectionPulseDv = 1.0;

	private const double PulseCompletionDv = 0.05;

	private const double MinimumUsefulCorrectionDv = 0.05;

	private const double PostBurnPredictionSettlingTime = 0.75;

	private const double CandidateDirectionAgreementAngle = 20.0;

	private const int RequiredCandidatePredictions = 2;

	private bool _courseCorrectionBurning;

	private bool _waitingForPostBurnPrediction;

	private double _remainingPulseDv;

	private Vector3d _pulseDirection;

	private long _predictionVersionAtPulseCompletion = -1L;

	private double _postBurnCompletionUT;

	private bool _hasPendingCorrection;

	private Vector3d _pendingCorrection;

	private int _pendingPredictionCount;

	private bool _hasLastPulseDirection;

	private Vector3d _lastPulseDirection;

	public override string TraceDetails
	{
		get
		{
			//IL_0028: Unknown result type (might be due to invalid IL or missing references)
			//IL_0038: Unknown result type (might be due to invalid IL or missing references)
			double num = (Core.Landing.PredictionReady ? Vector3d.Distance(Core.Target.GetPositionTargetPosition(), Core.Landing.LandingSite) : double.NaN);
			double lastCourseCorrectionDownrangeError = Core.Landing.LastCourseCorrectionDownrangeError;
			double lastCourseCorrectionLongBias = Core.Landing.LastCourseCorrectionLongBias;
			return $" targetError={num:F1} downrangeError={lastCourseCorrectionDownrangeError:F1} longBias={lastCourseCorrectionLongBias:F1} " + $"handoffLimit={Core.Landing.LastCourseCorrectionHandoffLimit:F1} pulseDv={_remainingPulseDv:F3} " + $"pulseDir=({_pulseDirection.x:F4},{_pulseDirection.y:F4},{_pulseDirection.z:F4}) " + $"burning={_courseCorrectionBurning} waiting={_waitingForPostBurnPrediction} " + $"waitForPredictionVersion={_predictionVersionAtPulseCompletion} " + $"pendingPredictions={_pendingPredictionCount}";
		}
	}

	private double CompletionError => Math.Max(200.0, base.MainBody.Radius * 0.0005);

	private double DownrangeCaptureDistance => Math.Max(100.0, base.MainBody.Radius * 0.005);

	private double MaximumDownrangeHandoffDistance
	{
		get
		{
			double num = Math.Max(0.1, base.VesselState.LimitedMaxThrustAcceleration);
			double val = base.VesselState.SpeedSurface * base.VesselState.SpeedSurface / (2.0 * num);
			return Math.Max(DownrangeCaptureDistance, Math.Min(base.MainBody.Radius * 0.01, val));
		}
	}

	public CourseCorrection(MechJebCore core)
		: base(core)
	{
	}

	public override AutopilotStep Drive(FlightCtrlState s)
	{
		//IL_0053: Unknown result type (might be due to invalid IL or missing references)
		//IL_0063: Unknown result type (might be due to invalid IL or missing references)
		//IL_02cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0294: Unknown result type (might be due to invalid IL or missing references)
		//IL_0299: Unknown result type (might be due to invalid IL or missing references)
		//IL_02b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_01b7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01bc: Unknown result type (might be due to invalid IL or missing references)
		//IL_0228: Unknown result type (might be due to invalid IL or missing references)
		//IL_0229: Unknown result type (might be due to invalid IL or missing references)
		//IL_01fa: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ff: Unknown result type (might be due to invalid IL or missing references)
		//IL_0211: Unknown result type (might be due to invalid IL or missing references)
		//IL_0212: Unknown result type (might be due to invalid IL or missing references)
		//IL_025e: Unknown result type (might be due to invalid IL or missing references)
		if (!Core.Landing.PredictionReady)
		{
			return this;
		}
		if (Core.Landing.DeployChutes && Core.Landing.ParachutesDeployable())
		{
			Core.Landing.ControlParachutes();
		}
		double num = Vector3d.Distance(Core.Target.GetPositionTargetPosition(), Core.Landing.LandingSite);
		if (num < CompletionError)
		{
			Core.Thrust.TargetThrottle = 0f;
			if (Core.Landing.RCSAdjustment)
			{
				Core.RCS.Enabled = true;
			}
			return new CoastToDeceleration(Core);
		}
		if (base.VesselState.AltitudeASL < Core.Landing.DecelerationEndAltitude() + 5.0)
		{
			return new DecelerationBurn(Core);
		}
		if (base.VesselState.ParachuteDeployed)
		{
			Core.Thrust.TargetThrottle = 0f;
			return new CoastToDeceleration(Core);
		}
		if (base.VesselState.DragAcceleration > 0.1)
		{
			return new CoastToDeceleration(Core);
		}
		if (_waitingForPostBurnPrediction)
		{
			Core.Thrust.TargetThrottle = 0f;
			if (Core.Landing.PredictionVersion <= _predictionVersionAtPulseCompletion || Core.Landing.Prediction.InputUT < _postBurnCompletionUT + 0.75)
			{
				return this;
			}
			Vector3d val = Core.Landing.ComputeCourseCorrection(allowPrograde: true, DownrangeCaptureDistance, MaximumDownrangeHandoffDistance);
			if (((Vector3d)(ref val)).magnitude <= 0.05)
			{
				return new CoastToDeceleration(Core);
			}
			_predictionVersionAtPulseCompletion = Core.Landing.PredictionVersion;
			if (_hasPendingCorrection && Vector3d.Angle(_pendingCorrection, val) <= 20.0)
			{
				_pendingCorrection = val;
				_pendingPredictionCount++;
			}
			else
			{
				_pendingCorrection = val;
				_pendingPredictionCount = 1;
				_hasPendingCorrection = true;
			}
			if (_pendingPredictionCount < 2)
			{
				return this;
			}
			_waitingForPostBurnPrediction = false;
			_courseCorrectionBurning = false;
			_hasPendingCorrection = false;
			BeginPulse(_pendingCorrection, num);
		}
		else if (_remainingPulseDv <= 0.0)
		{
			Vector3d deltaV = Core.Landing.ComputeCourseCorrection(allowPrograde: true, DownrangeCaptureDistance, MaximumDownrangeHandoffDistance);
			if (((Vector3d)(ref deltaV)).magnitude <= 0.05)
			{
				return new CoastToDeceleration(Core);
			}
			BeginPulse(deltaV, num);
		}
		Core.Attitude.attitudeTo(_pulseDirection, AttitudeReference.INERTIAL, Core.Landing);
		if (Core.Attitude.attitudeAngleFromTarget() < 2.0)
		{
			_courseCorrectionBurning = true;
		}
		else if (Core.Attitude.attitudeAngleFromTarget() > 30.0)
		{
			_courseCorrectionBurning = false;
		}
		if (_courseCorrectionBurning)
		{
			Core.Thrust.ThrustForDv(_remainingPulseDv, 0.5);
			_remainingPulseDv -= base.VesselState.CurrentThrustAcceleration * (double)TimeWarp.fixedDeltaTime;
			if (_remainingPulseDv <= 0.05)
			{
				Core.Thrust.TargetThrottle = 0f;
				_remainingPulseDv = 0.0;
				_courseCorrectionBurning = false;
				_waitingForPostBurnPrediction = true;
				_predictionVersionAtPulseCompletion = Core.Landing.PredictionVersion;
				_postBurnCompletionUT = base.VesselState.Time;
				_hasPendingCorrection = false;
				_pendingPredictionCount = 0;
			}
		}
		else
		{
			Core.Thrust.TargetThrottle = 0f;
		}
		return this;
	}

	private void BeginPulse(Vector3d deltaV, double targetError)
	{
		//IL_0058: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0067: Unknown result type (might be due to invalid IL or missing references)
		//IL_006c: Unknown result type (might be due to invalid IL or missing references)
		double num = 1.0;
		double num2 = Math.Max(250.0, base.MainBody.Radius * 0.01);
		if (targetError < num2)
		{
			num = 0.1;
		}
		else if (targetError < 4.0 * num2)
		{
			num = 0.25;
		}
		Vector3d normalized = ((Vector3d)(ref deltaV)).normalized;
		if (_hasLastPulseDirection && Vector3d.Angle(_lastPulseDirection, normalized) > 90.0)
		{
			num = Math.Min(num, 0.1);
		}
		_remainingPulseDv = Math.Min(((Vector3d)(ref deltaV)).magnitude, num);
		_pulseDirection = normalized;
		_lastPulseDirection = normalized;
		_hasLastPulseDirection = true;
		base.Status = Localizer.Format("#MechJeb_LandingGuidance_Status3", new string[1] { ((Vector3d)(ref deltaV)).magnitude.ToString("F1") });
	}
}
