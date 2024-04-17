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

public class KPMatch extends BaseMatch{
    public KPMatch(KMM KMM) {
        super(KMM);
    }

    @Override
    public List<SequenceState<State, Observation, Path>> match(List<ObservationWithCandidateStates> timeSteps) {
        System.out.println("Using rule-based method");
        int keypointNum = 0;
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
        ObservationWithCandidateStates dstTimeStep = timeSteps.get(timeSteps.size()-1);
        List<ObservationWithCandidateStates> tempKeyPoints = new ArrayList<>();
        Map<State,Double> forwardLenMap = new HashMap<>();
        Map<State,SequenceState<State, Observation, Path>> forwardPathMap = new HashMap<>();
        Map<State,State> forwardLastStateMap = new HashMap<>();
        Path bestPathToPrev = null;
        Path bestPathToDst = null;
        State bestDstState = null;
        ObservationWithCandidateStates prevKeypoint = null;
        for(int i=0;i<timeSteps.size()-1;i++){
            if(keypointNum >= KMM.mmConfig.getMaxKeypointNum()+1){
                break;
            }
            ObservationWithCandidateStates timeStep = timeSteps.get(i);
            //check if this observation is far away from current path
            List<EdgeIteratorState> allEdges = new ArrayList<>();
            if(bestPathToPrev!=null){
                allEdges.addAll(bestPathToPrev.calcEdges());
            }
            if(bestPathToDst!=null){
                allEdges.addAll(bestPathToDst.calcEdges());
            }
            if(!pointNearEdges(timeStep.observation, KMM.mmConfig.getKpDisThreshold(),allEdges)){
                tempKeyPoints.add(timeStep);
            }else{
                tempKeyPoints.clear();
            }
            //Check if consecutive observations are deviant keypoints.
            if(tempKeyPoints.size()>= KMM.mmConfig.getKpConnumThreshold()||i==0){
                keypointNum+=1;
                KMM.keypoints.add(i);
                //choose the last deviant point as keypoint
                ObservationWithCandidateStates keyPoint = tempKeyPoints.get(tempKeyPoints.size()-1);
                double minTotalLen = Double.MAX_VALUE;
                for(State candidate:keyPoint.candidates){
                    //calculate the min distance to prev
                    if(prevKeypoint==null){
                        forwardLenMap.put(candidate, 0.0);
                        forwardPathMap.put(candidate,new SequenceState<>(candidate,keyPoint.observation,null));
                    }else{
                        double minLenToLast = Double.MAX_VALUE;
                        SequenceState<State, Observation, Path> bestSequence = null;
                        State bestLastState = null;
                        for(State lastCandidate:prevKeypoint.candidates){
                            if(!forwardLenMap.containsKey(lastCandidate)){
                                continue;
                            }
                            Path path = KMM.router.calcPath(lastCandidate.getSnap().getClosestNode(), candidate.getSnap().getClosestNode(), EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
                            double pathDistance = getPathWeight(path);
                            if(pathDistance + forwardLenMap.get(lastCandidate) < minLenToLast){
                                minLenToLast = pathDistance + forwardLenMap.get(lastCandidate);
                                bestSequence = new SequenceState<>(candidate,keyPoint.observation,path);
                                bestLastState = lastCandidate;
                            }
                        }
                        forwardLenMap.put(candidate,minLenToLast);
                        forwardPathMap.put(candidate,bestSequence);
                        forwardLastStateMap.put(candidate,bestLastState);
                    }
                    //calculate the min distance to dst
                    double minDisToDst = Double.MAX_VALUE;
                    Path minPathToDst = null;
                    State tempDst = null;
                    for(State dstCandidate:dstTimeStep.candidates){
                        Path path = KMM.router.calcPath(candidate.getSnap().getClosestNode(), dstCandidate.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
                        double pathDistance = getPathWeight(path);
                        if(pathDistance< minDisToDst){
                            minDisToDst = pathDistance;
                            minPathToDst = path;
                            tempDst = dstCandidate;
                        }
                    }
                    if(minDisToDst + forwardLenMap.get(candidate) < minTotalLen){
                        minTotalLen = minDisToDst + forwardLenMap.get(candidate);
                        bestPathToPrev = forwardPathMap.get(candidate).transitionDescriptor;
                        bestPathToDst = minPathToDst;
                        bestDstState = tempDst;
                        forwardLastStateMap.put(bestDstState,candidate);
                    }
                }
                prevKeypoint = keyPoint;
                tempKeyPoints.clear();
            }
        }
        result.add(new SequenceState<>(bestDstState,timeSteps.get(timeSteps.size()-1).observation,bestPathToDst));
        State thisCandidate = forwardLastStateMap.get(bestDstState);
        while(forwardPathMap.containsKey(thisCandidate)){
            result.add(forwardPathMap.get(thisCandidate));
            thisCandidate = forwardLastStateMap.get(thisCandidate);
        }
        Collections.reverse(result);
        this.KMM.keypointNums = keypointNum;
        return result;
    }

    public boolean pointNearEdges(Observation obs,double threshold,List<EdgeIteratorState> edges){
        for(EdgeIteratorState e:edges){
            double n1lat = KMM.queryGraph.getNodeAccess().getLat(e.getBaseNode());
            double n1lon = KMM.queryGraph.getNodeAccess().getLon(e.getBaseNode());
            double n2lat = KMM.queryGraph.getNodeAccess().getLat(e.getAdjNode());
            double n2lon = KMM.queryGraph.getNodeAccess().getLon(e.getAdjNode());
            double dis = calcNormalizedEdgeDistance(obs.getPoint().lat, obs.getPoint().lon,n1lat,n1lon,n2lat,n2lon);
            dis = KMM.distanceCalc.calcDenormalizedDist(dis);
            if (dis < threshold){
                return true;
            }
        }
        return false;
    }

    public double getPathWeight(Path path){
        if(KMM.mmConfig.getWeightType() == 0){
            return getPathDistance(path);
        }else if(KMM.mmConfig.getWeightType() == 1) {
            return getPathTime(path);
        }else{
            throw new IllegalArgumentException("Invalid weight method");
        }
    }
    public double getPathDistance(Path path){
        double result = 0;
        for(EdgeIteratorState e:path.calcEdges()){
            result += e.getDistance();
        }
        return result;
    }

    public double getPathTime(Path path){
        double result = 0;
        for(EdgeIteratorState e:path.calcEdges()){
            result += KMM.weighting.calcEdgeMillis(e, false);
        }
        return result;
    }
}
