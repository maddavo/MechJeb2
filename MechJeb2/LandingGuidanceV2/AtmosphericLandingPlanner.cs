using System;
using MechJebLib.HoverslamSimulation;
using UnityEngine;

namespace MuMech
{
    /// <summary>
    /// Converts the independent re-entry simulation into a conservative V2
    /// entry-corridor decision.  It intentionally reports a corridor and an
    /// uncertainty rather than claiming an atmospheric endpoint is exact.
    /// </summary>
    public static class AtmosphericLandingPlanner
    {
        public static AtmosphericLandingPlan Plan(LandingGuidanceV2Snapshot snapshot, ReentrySimulation.Result estimate)
        {
            if (snapshot == null || snapshot.Body == null || !snapshot.Body.atmosphere)
                return new AtmosphericLandingPlan(snapshot?.Version ?? -1, AtmosphericLandingPlanState.NotApplicable,
                    double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                    "An atmospheric body snapshot is required.");

            if (estimate == null || estimate.Body != snapshot.Body)
                return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.WaitingForEstimate,
                    double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                    "Waiting for a fresh atmospheric trajectory estimate from this vessel state.");

            if (estimate.Outcome != ReentrySimulation.Outcome.LANDED)
                return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.Rejected,
                    double.NaN, double.NaN, double.NaN, double.NaN, double.NaN,
                    "The atmospheric estimator did not produce a landing outcome: " + estimate.Outcome + ".");

            Vector3d predicted = snapshot.Body.GetWorldSurfacePosition(estimate.EndPosition.Latitude, estimate.EndPosition.Longitude, estimate.EndASL);
            Vector3d target = snapshot.Body.GetWorldSurfacePosition(snapshot.TargetLatitude, snapshot.TargetLongitude, 0);
            double targetError = Vector3d.Distance(predicted, target);
            double endpointSpeed = estimate.ReferenceFrame == null
                ? double.NaN
                : estimate.ReferenceFrame.WorldVelocityAtCurrentTime(estimate.EndVelocity).magnitude;

            // The re-entry simulation is model based but not exact.  Use the
            // simulated drag exposure and current endpoint scale to calculate a
            // conservative corridor, then preserve a powered terminal reserve.
            double uncertainty = Math.Max(500.0, Math.Min(15000.0,
                250.0 + 25.0 * Math.Max(0, estimate.MaxDragGees) + 0.10 * Math.Max(0, endpointSpeed)));
            double corridor = Math.Max(3000.0, 3.0 * uncertainty);
            double radius = snapshot.Body.Radius + Math.Max(0, estimate.EndASL);
            double gravity = snapshot.Body.gravParameter / (radius * radius);
            AirlessLandingBudget terminalBudget = AirlessLandingBudget.For(0, endpointSpeed,
                snapshot.MaximumAcceleration, gravity, uncertainty, corridor);
            double margin = snapshot.AvailableDeltaV - terminalBudget.Total;
            if (targetError > corridor)
                return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.Rejected,
                    targetError, corridor, uncertainty, terminalBudget.Reserve, margin,
                    "The predicted atmospheric endpoint is outside the robust entry corridor; V2 will not present it as a touchdown solution.");
            if (!terminalBudget.Fits(snapshot.AvailableDeltaV))
                return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.Rejected,
                    targetError, corridor, uncertainty, terminalBudget.Reserve, margin,
                    "Usable delta-V does not retain the powered terminal reserve required by the atmospheric plan.");
            return new AtmosphericLandingPlan(snapshot.Version, AtmosphericLandingPlanState.Candidate,
                targetError, corridor, uncertainty, terminalBudget.Reserve, margin,
                "Atmospheric entry corridor is feasible; endpoint remains an uncertainty-bounded estimate until local terminal guidance.");
        }
    }
}
