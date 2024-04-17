package matchAlgo;

import com.graphhopper.routing.Path;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import entities.Observation;
import entities.ObservationWithCandidateStates;
import entities.SequenceState;
import entities.State;
import kmm.KMM;

import java.util.*;

import static java.lang.Math.*;

public class STMatch extends BaseMatch{

    public STMatch(KMM KMM) {
        super(KMM);
    }

    @Override
    public List<SequenceState<State, Observation, Path>> match(List<ObservationWithCandidateStates> timeSteps) {
        System.out.println("Using STMatching");
        long t1 = System.currentTimeMillis();
        Map<State, Double> sd = new HashMap<>();
        Map<State, State> bl = new HashMap<>();
        Map<State,SequenceState<State, Observation, Path>> bd = new HashMap<>();
        ObservationWithCandidateStates lastTimeStep = null;
        for(ObservationWithCandidateStates timeStep : timeSteps){
            boolean allCandidateUnreachable = true;
            for(State candidate:timeStep.candidates){
                if(lastTimeStep==null){
                    sd.put(candidate, 0.0);
                    allCandidateUnreachable = false;
                }else{
                    double maxScore = 0;
                    State bestLastCandidate = null;
                    Path bestPath = null;
                    for(State lastCandidate:lastTimeStep.candidates){
                        if(!sd.containsKey(lastCandidate)){
                            continue;
                        }
                        Path path = KMM.router.calcPath(lastCandidate.getSnap().getClosestNode(), candidate.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE, EdgeIterator.ANY_EDGE);
                            double Nscore = 1/(KMM.mmConfig.getMeasurementErrorSigma()*sqrt(2*PI))*exp(-pow(calcNormalizedDist(timeStep.observation.getPoint().getLat(),timeStep.observation.getPoint().getLon(),candidate.getSnap().getSnappedPoint().getLat(),candidate.getSnap().getSnappedPoint().getLon()),2)/(2*pow(KMM.mmConfig.getMeasurementErrorSigma(),2)));
                            double Vscore = calcNormalizedDist(timeStep.observation.getPoint().getLat(),timeStep.observation.getPoint().getLon(),lastTimeStep.observation.getPoint().getLat(),lastTimeStep.observation.getPoint().getLon())/getPathDistance(path);
                            double tScore = 0;
                            List<Double> edgeVelocity = new ArrayList<>();
                            List<Double> avrVelocity = new ArrayList<>();
                            for(EdgeIteratorState e:path.calcEdges()){
                                edgeVelocity.add(e.getDistance()/ KMM.weighting.calcEdgeMillis(e, false));
                            }
                            for(int i = 0;i<edgeVelocity.size();i++){
                                avrVelocity.add(getPathDistance(path)/(timeStep.observation.getTime().getTime() - lastTimeStep.observation.getTime().getTime()));
                            }
                            double dotProduct = 0.0;
                            double normA = 0.0;
                            double normB = 0.0;
                            for (int i = 0; i < edgeVelocity.size(); i++) {
                                dotProduct += edgeVelocity.get(i) * avrVelocity.get(i);
                                normA += Math.pow(edgeVelocity.get(i), 2);
                                normB += Math.pow(avrVelocity.get(i), 2);
                            }
                            tScore = dotProduct / (Math.sqrt(normA) * Math.sqrt(normB));
                            double score = Nscore * Vscore * tScore;
                            if(score + sd.get(lastCandidate) > maxScore){
                                maxScore = score + sd.get(lastCandidate);
                                bestLastCandidate = lastCandidate;
                                bestPath = path;
                            }
                    }
                    if(bestPath==null){
                        continue;
                    }
                    //Filter out paths with obviously unreasonable speeds
                    if(!KMM.mmConfig.isUseVelocity() ||
                            //v < v_threshold
                            getPathDistance(bestPath) * 1000/ (timeStep.observation.getTime().getTime() - lastTimeStep.observation.getTime().getTime())
                                    < KMM.mmConfig.getVelocityThreshold()){
                        sd.put(candidate,maxScore);
                        bl.put(candidate,bestLastCandidate);
                        bd.put(candidate,new SequenceState<>(candidate,timeStep.observation,bestPath));
                        allCandidateUnreachable = false;
                    }
                }
            }
            if(!allCandidateUnreachable){
                lastTimeStep = timeStep;
            }
        }
        State minLastState = null;
        Double minTotalLen = Double.MAX_VALUE;
        //
        for(State candidate:lastTimeStep.candidates){
            if(sd.containsKey(candidate) && sd.get(candidate) < minTotalLen){
                minTotalLen = sd.get(candidate);
                minLastState = candidate;
            }
        }
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
        State thisCandidate = minLastState;
        while(bl.containsKey(thisCandidate)){
            result.add(bd.get(thisCandidate));
            thisCandidate = bl.get(thisCandidate);
        }
        Collections.reverse(result);
        long t2 = System.currentTimeMillis();
//        System.out.println("SPMatch time: "+(t2-t1));
        return result;
    }

    public double getPathDistance(Path path){
        double result = 0;
        for(EdgeIteratorState e:path.calcEdges()){
            result += e.getDistance();
        }
        return result;
    }
}
