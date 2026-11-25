using System;
using System.Collections.Generic;

namespace MuMech.Landing
{
    // Optional finite steering response; the published baseline is never replaced
    // by a target-reaching trial. Units of response are metres per vector ratio.
    internal sealed class BrakingEndpointResponse
    {
        internal const double DefaultCorrectionLimit = 1;
        internal readonly Vector3d[] Directions;
        internal readonly List<AirlessTargetAwareState>[] Paths;
        internal readonly double Horizon;
        internal readonly double MaximumRatio;
        internal readonly AirlessTargetAwareState BaselineContact;
        internal readonly AirlessTargetAwareState[] Contacts;
        internal readonly double[] FinalDescentUTs;
        internal readonly bool ImpactBaseline;
        private readonly Vector3d[] _impactCommands;
        internal bool Validated;
        internal Vector3d RadialResponse, NormalResponse;
        internal Vector3d SteeringCommand;
        internal double PredictedResidual;

        private BrakingEndpointResponse(Vector3d[] directions,
            List<AirlessTargetAwareState>[] paths, double[] finalDescentUTs, double horizon, double maximumDeltaV,
            AirlessTargetAwareState baseline, Vector3d[] impactCommands = null)
        {
            Directions = directions; Paths = paths; FinalDescentUTs=finalDescentUTs;
            Horizon = horizon; MaximumRatio=maximumDeltaV; BaselineContact = baseline;
            Contacts = new AirlessTargetAwareState[paths.Length];
            _impactCommands = impactCommands; ImpactBaseline = impactCommands != null;
        }

        // An unsafe baseline is still information. Trial commands are separate
        // from that baseline and are never published as a landing trajectory.
        internal static BrakingEndpointResponse RunImpact(AirlessTargetAwareSnapshot s,
            AirlessTargetAwareState contact, Action cancellation)
        {
            const double horizon = 2;
            const double ratio = .1; // existing fallback steering authority
            if (!s.DecelerationAlreadyTriggered || s.TerminalPhase != 0 || s.CoastToDeceleration ||
                s.MinimumCommandThrottle > 0 || s.TerminalAngularAcceleration <= 0 ||
                contact.UT <= s.StateUT + horizon) return null;
            Vector3d surface = s.Velocity - Vector3d.Cross(s.AngularVelocity, s.Position);
            Vector3d radial = Vector3d.Exclude(surface, s.Position.normalized).normalized;
            Vector3d normal = -Vector3d.Cross(radial, surface.normalized);
            if (radial.sqrMagnitude < .5 || normal.sqrMagnitude < .5) return null;
            var paths = new List<List<AirlessTargetAwareState>>();
            var commands = new List<Vector3d>();
            Vector3d target = (s.BodyRadius + s.TargetTerrainASL) *
                AirlessTargetAwareSimulation.SurfaceDirection(s.TargetLatitude, s.TargetLongitude, s);
            Vector3d error = Vector3d.Exclude(target, target - BodyPoint(s, contact));
            // Bound each refresh to the dominant error axis. Re-evaluate both
            // axes on the next fresh snapshot; do not build eight whole paths
            // for a negligible crossrange error while the ship is burning.
            Vector3d axis = Math.Abs(Vector3d.Dot(error, normal)) > Math.Abs(Vector3d.Dot(error, radial)) ? normal : radial;
            foreach (Vector3d command in new[] { ratio * axis, -ratio * axis })
            {
                var pair = new List<List<AirlessTargetAwareState>>();
                foreach (double rate in new[] { AirlessTargetAwareSimulation.NominalTerminalTurnRate,
                    AirlessTargetAwareSimulation.SlowTerminalTurnRate })
                {
                    cancellation?.Invoke();
                    var burn = AirlessTargetAwareSimulation.RunBrakingSteeringProbe(s, command,
                        s.StateUT + horizon, rate / AirlessTargetAwareSimulation.NominalTerminalTurnRate);
                    if (!burn.ReachedHandoff || burn.AttitudeUncertain || !burn.SteeringReserveMaintained) break;
                    var terminal = AirlessTargetAwareSimulation.RunNominalTerminal(s, burn, s.TargetTerrainASL, 1, rate);
                    var path = new List<AirlessTargetAwareState>(burn.Trajectory);
                    path.AddRange(terminal.Trajectory);
                    pair.Add(path);
                }
                if (pair.Count != 2) continue;
                paths.AddRange(pair); commands.Add(command);
            }
            return commands.Count == 0 ? null : new BrakingEndpointResponse(new[] { radial, normal },
                paths.ToArray(), new double[paths.Count], horizon, ratio, contact, commands.ToArray());
        }

        internal static BrakingEndpointResponse Run(AirlessTargetAwareSnapshot s,
            AirlessTargetAwareOutput baseline, AirlessNominalTerminalOutput terminal,
            double terrain, double terrainDepth, Action cancellation)
        {
            // Three bounded amplitudes, no terrain search or target-reaching
            // brake-time optimization. Smaller probes retain thrust reserve.
            foreach(double scale in new[] {1.0,.5,.25})
            {
                cancellation?.Invoke();
                var response=RunScaled(s,baseline,terminal,terrain,terrainDepth,cancellation,scale);
                if(response!=null)return response;
            }
            return null;
        }

        private static BrakingEndpointResponse RunScaled(AirlessTargetAwareSnapshot s,
            AirlessTargetAwareOutput baseline, AirlessNominalTerminalOutput terminal,
            double terrain, double terrainDepth, Action cancellation,double scale)
        {
            if (!s.DecelerationAlreadyTriggered || s.TerminalPhase != 0 || s.CoastToDeceleration ||
                s.InitialAppliedThrottle <= 0 || s.MinimumCommandThrottle>0 ||
                s.TerminalAngularAcceleration<=0 || baseline == null || !baseline.ReachedHandoff ||
                !terminal.HorizontalKill.ReachedFinalDescent || terminal.FinalDescent == null ||
                !terminal.FinalDescent.ReachedTerrain) return null;
            Vector3d surface = s.Velocity - Vector3d.Cross(s.AngularVelocity,s.Position);
            Vector3d radial = Vector3d.Exclude(surface,s.Position.normalized).normalized;
            Vector3d normal = -Vector3d.Cross(radial,surface.normalized);
            if (radial.sqrMagnitude < .5 || normal.sqrMagnitude < .5) return null;
            var paths = new List<AirlessTargetAwareState>[6];
            var handoffs = new double[6];
            var directions = new[] { radial,normal };
            double horizon=Math.Min(2,Math.Max(0,baseline.End.UT-s.StateUT));
            // Enhanced authority belongs to early braking. Leave the existing
            // final two seconds/handoff steering untouched.
            if(horizon<2)return null;
            double probeRatio=scale*MarginRatio(s.InitialAppliedThrottle,s.MaximumCommandThrottle);
            if (probeRatio<=0 || horizon<s.MinDt) return null;
            for (int i=0;i<4;i++)
            {
                cancellation?.Invoke();
                double sign = i%2 == 0 ? 1 : -1;
                var changed=s;
                var burn = AirlessTargetAwareSimulation.RunBrakingSteeringProbe(changed,
                    sign*probeRatio*directions[i/2],s.StateUT+horizon);
                if (!burn.ReachedHandoff || burn.AttitudeUncertain || !burn.SteeringReserveMaintained) return null;
                var end = AirlessTargetAwareSimulation.RunNominalTerminal(changed,burn,terrain,terrainDepth);
                if (!end.HorizontalKill.ReachedFinalDescent || end.FinalDescent == null ||
                    !end.FinalDescent.ReachedTerrain) return null;
                paths[i] = new List<AirlessTargetAwareState>(burn.Trajectory);
                paths[i].AddRange(end.Trajectory);
                handoffs[i]=end.HorizontalKillEndUT;
            }
            var result=new BrakingEndpointResponse(directions,paths,handoffs,horizon,probeRatio,terminal.End);
            for(int i=0;i<4;i++)result.Contacts[i]=paths[i][paths[i].Count-1];
            if(!result.Resolve(s,terminal.End,false))return null;
            Vector3d center=BodyPoint(s,terminal.End);
            Vector3d target=(s.BodyRadius+terrain)*AirlessTargetAwareSimulation.SurfaceDirection(
                s.TargetLatitude,s.TargetLongitude,s);
            Vector3d error=Vector3d.Exclude(center,target-center);
            error*=error.magnitude/(error.magnitude+2*terrainDepth);
            if(!TryCorrection(error,result.RadialResponse,result.NormalResponse,
                radial,normal,out result.SteeringCommand,probeRatio))return null;
            cancellation?.Invoke();
            var candidate=AirlessTargetAwareSimulation.RunBrakingSteeringProbe(s,
                result.SteeringCommand,s.StateUT+horizon);
            if(!candidate.ReachedHandoff || candidate.AttitudeUncertain || !candidate.SteeringReserveMaintained)return null;
            var candidateEnd=AirlessTargetAwareSimulation.RunNominalTerminal(s,candidate,terrain,terrainDepth);
            if(!candidateEnd.HorizontalKill.ReachedFinalDescent || candidateEnd.FinalDescent==null ||
                !candidateEnd.FinalDescent.ReachedTerrain)return null;
            paths[4]=new List<AirlessTargetAwareState>(candidate.Trajectory);
            paths[4].AddRange(candidateEnd.Trajectory);
            handoffs[4]=candidateEnd.HorizontalKillEndUT;
            cancellation?.Invoke();
            var delayed=AirlessTargetAwareSimulation.RunBrakingSteeringProbe(s,result.SteeringCommand,
                s.StateUT+horizon,AirlessTargetAwareSimulation.SlowTerminalTurnRate/
                    AirlessTargetAwareSimulation.NominalTerminalTurnRate);
            if(!delayed.ReachedHandoff || !delayed.SteeringReserveMaintained)return null;
            var delayedEnd=AirlessTargetAwareSimulation.RunNominalTerminal(s,delayed,terrain,terrainDepth,
                AirlessTargetAwareSimulation.SlowTerminalTurnRate);
            if(!delayedEnd.HorizontalKill.ReachedFinalDescent || delayedEnd.FinalDescent==null ||
                !delayedEnd.FinalDescent.ReachedTerrain)return null;
            paths[5]=new List<AirlessTargetAwareState>(delayed.Trajectory);
            paths[5].AddRange(delayedEnd.Trajectory);
            handoffs[5]=delayedEnd.HorizontalKillEndUT;
            result.Validated=false;
            return result;
        }

        internal bool Resolve(AirlessTargetAwareSnapshot s, AirlessTargetAwareState baseline,bool validateCommand=true)
        {
            if (ImpactBaseline) return ResolveImpact(s);
            for(int i=0;i<(validateCommand?6:4);i++)
                if(!Finite(Contacts[i].Position.x) || !Finite(Contacts[i].Position.y) ||
                    !Finite(Contacts[i].Position.z) || !Finite(Contacts[i].UT) ||
                    Contacts[i].Position.magnitude<s.BodyRadius+s.MinimumTerrainASL ||
                    Contacts[i].UT<s.StateUT) return false;
            Vector3d center = BodyPoint(s,baseline);
            var responses = new Vector3d[2];
            for (int axis=0;axis<2;axis++)
            {
                Vector3d plus = Vector3d.Exclude(center,BodyPoint(s,Contacts[2*axis])-center);
                Vector3d minus = Vector3d.Exclude(center,center-BodyPoint(s,Contacts[2*axis+1]));
                if (plus.magnitude < .01 || minus.magnitude < .01 ||
                    Vector3d.Dot(plus,minus) <= 0 ||
                    (plus-minus).magnitude > .5*(plus.magnitude+minus.magnitude)) return false;
                responses[axis] = (plus+minus)/(2*MaximumRatio);
            }
            if (!WellConditioned(responses[0],responses[1])) return false;
            RadialResponse=responses[0]; NormalResponse=responses[1];
            if(validateCommand)
            {
                Vector3d target=(s.BodyRadius+s.TargetTerrainASL)*AirlessTargetAwareSimulation.SurfaceDirection(
                    s.TargetLatitude,s.TargetLongitude,s);
                PredictedResidual=Vector3d.Exclude(target,BodyPoint(s,Contacts[4])-target).magnitude;
                if(PredictedResidual>=Vector3d.Exclude(target,center-target).magnitude)return false;
                Validated=true;
            }
            return true;
        }

        private bool ResolveImpact(AirlessTargetAwareSnapshot s)
        {
            Vector3d target = (s.BodyRadius + s.TargetTerrainASL) *
                AirlessTargetAwareSimulation.SurfaceDirection(s.TargetLatitude, s.TargetLongitude, s);
            double baselineMiss = Vector3d.Exclude(target, BodyPoint(s, BaselineContact) - target).magnitude;
            double baselineSpeed = ContactSpeed(s, BaselineContact);
            if (!Finite(baselineMiss) || !Finite(baselineSpeed)) return false;
            double best = baselineMiss;
            for (int i = 0; i < _impactCommands.Length; ++i)
            {
                var first = Contacts[2 * i]; var slow = Contacts[2 * i + 1];
                if (!Finite(first.UT) || !Finite(slow.UT) || first.UT <= s.StateUT + Horizon ||
                    slow.UT <= s.StateUT + Horizon || !Finite(ContactSpeed(s, first)) ||
                    !Finite(ContactSpeed(s, slow)) || ContactSpeed(s, first) > baselineSpeed ||
                    ContactSpeed(s, slow) > baselineSpeed) continue;
                double miss = Math.Max(Vector3d.Exclude(target, BodyPoint(s, first) - target).magnitude,
                    Vector3d.Exclude(target, BodyPoint(s, slow) - target).magnitude);
                if (!Finite(miss) || miss >= best) continue;
                best = miss; SteeringCommand = _impactCommands[i];
            }
            PredictedResidual = best;
            Validated = best < baselineMiss && SteeringCommand.sqrMagnitude > 0;
            return Validated;
        }

        private static double ContactSpeed(AirlessTargetAwareSnapshot s, AirlessTargetAwareState contact) =>
            (contact.Velocity - Vector3d.Cross(s.AngularVelocity, contact.Position)).magnitude;

        internal static Vector3d BodyPoint(AirlessTargetAwareSnapshot s, AirlessTargetAwareState state)
        {
            var a=AirlessTargetAwareSimulation.ToAbsolute(state.Position,state.UT,s);
            return a.Radius*AirlessTargetAwareSimulation.SurfaceDirection(a.Latitude,a.Longitude,s);
        }

        internal static bool WellConditioned(Vector3d a,Vector3d b)
        {
            double aa=a.sqrMagnitude,bb=b.sqrMagnitude,ab=Vector3d.Dot(a,b);
            return Finite(aa) && Finite(bb) && Finite(ab) && aa>1e-6 && bb>1e-6 &&
                aa*bb-ab*ab > 1e-4*aa*bb;
        }

        internal static bool TryCorrection(Vector3d error,Vector3d a,Vector3d b,
            Vector3d radial,Vector3d normal,out Vector3d correction,double maximumMagnitude=DefaultCorrectionLimit)
        {
            correction=Vector3d.zero;
            if (!WellConditioned(a,b) || !Finite(maximumMagnitude) || maximumMagnitude<=0) return false;
            double aa=a.sqrMagnitude,bb=b.sqrMagnitude,ab=Vector3d.Dot(a,b);
            double ea=Vector3d.Dot(error,a),eb=Vector3d.Dot(error,b),det=aa*bb-ab*ab;
            double x=(ea*bb-eb*ab)/det,y=(eb*aa-ea*ab)/det;
            if (!Finite(x) || !Finite(y)) return false;
            correction=x*radial+y*normal;
            // Never extrapolate a large pulse from a small local derivative.
            if (correction.magnitude>maximumMagnitude) correction*=maximumMagnitude/correction.magnitude;
            return true;
        }

        internal static double MarginRatio(double nominalThrottle,double maximumThrottle)
        {
            if (!Finite(nominalThrottle) || !Finite(maximumThrottle) || nominalThrottle<=0 || maximumThrottle<=0) return 0;
            // Reserve ten percent of command authority. The existing 0.75
            // alignment gate bounds the permitted change from retrograde.
            double cosine=nominalThrottle/(.9*maximumThrottle);
            if(cosine>=1)return .1;
            cosine=Math.Max(.75,cosine);
            return Math.Max(.1,Math.Sqrt(1/(cosine*cosine)-1));
        }

        internal static bool FreshCorrectionForecast(ReentrySimulation.Outcome outcome, bool hasResponse,
            double age, double maximumAge) => hasResponse && Finite(age) && age >= 0 && age <= maximumAge &&
            (outcome == ReentrySimulation.Outcome.LANDED || outcome == ReentrySimulation.Outcome.IMPACT);

        private static bool Finite(double x)=>!double.IsNaN(x)&&!double.IsInfinity(x);
    }

    internal readonly struct PublishedBrakingResponse
    {
        internal readonly AbsoluteVector Radial,Normal,RadialResponse,NormalResponse,SteeringCommand;
        internal readonly double Horizon,MaximumRatio;
        internal PublishedBrakingResponse(AbsoluteVector radial,AbsoluteVector normal,
            AbsoluteVector radialResponse,AbsoluteVector normalResponse,AbsoluteVector steeringCommand,double horizon,double maximumRatio)
        { Radial=radial;Normal=normal;RadialResponse=radialResponse;NormalResponse=normalResponse;SteeringCommand=steeringCommand;Horizon=horizon;MaximumRatio=maximumRatio; }
    }
}
