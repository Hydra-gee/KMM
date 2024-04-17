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

public class SPMatch extends BaseMatch{

    public SPMatch(KMM KMM) {
        super(KMM);
    }

    @Override
    public List<SequenceState<State, Observation, Path>> match(List<ObservationWithCandidateStates> timeSteps) {
        System.out.println("Using SPMatch");
        //the distance of shortest path from the origin to each candidate state
        Map<State, Double> sd = new HashMap<>();
        //the best previous state of each candidate state
        Map<State, State> bl = new HashMap<>();
        //the corresponding path segment of each candidate state
        Map<State,SequenceState<State, Observation, Path>> bd = new HashMap<>();
        ObservationWithCandidateStates lastTimeStep = null;
        for(ObservationWithCandidateStates timeStep : timeSteps){
            boolean allCandidateUnreachable = true;
            for(State candidate:timeStep.candidates){
                if(lastTimeStep==null){
                    sd.put(candidate, 0.0);
                    allCandidateUnreachable = false;
                }else{
                    double minLen = Double.MAX_VALUE;
                    State bestLastCandidate = null;
                    Path bestPath = null;
                    for(State lastCandidate:lastTimeStep.candidates){
                        if(!sd.containsKey(lastCandidate)){
                            continue;
                        }
                        Path path = KMM.router.calcPath(lastCandidate.getSnap().getClosestNode(), candidate.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE, EdgeIterator.ANY_EDGE);
                        if(getPathDistance(path) + sd.get(lastCandidate) < minLen){
                            minLen = getPathDistance(path) + sd.get(lastCandidate);
                            bestLastCandidate = lastCandidate;
                            bestPath = path;
                        }
                    }
                    if(bestPath==null){
                        continue;
                    }
                    //Filter out paths with obviously unreasonable speeds
                    if(!KMM.mmConfig.isUseVelocity() ||
                            getPathDistance(bestPath) * 1000/ (timeStep.observation.getTime().getTime() - lastTimeStep.observation.getTime().getTime())
                                    < KMM.mmConfig.getVelocityThreshold()){
                        sd.put(candidate,minLen);
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
