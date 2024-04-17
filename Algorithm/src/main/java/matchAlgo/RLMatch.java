package matchAlgo;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.graphhopper.routing.Path;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import entities.*;
import kmm.KMM;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.*;

import static java.lang.Integer.parseInt;

public class RLMatch extends BaseMatch{

    public RLMatch(KMM KMM) {
        super(KMM);
    }

    @Override
    public List<SequenceState<State, Observation, Path>> match(List<ObservationWithCandidateStates> timeSteps) {
        String hostName = "localhost";
        int portNumber = 7878;
        List<SequenceState<State, Observation, Path>> result = new ArrayList<>();
        long socketTime = 0;
        try (
                Socket echoSocket = new Socket(hostName, portNumber);
                PrintWriter out =
                        new PrintWriter(echoSocket.getOutputStream(), true);
                BufferedReader in =
                        new BufferedReader(
                                new InputStreamReader(echoSocket.getInputStream()))
        ) {
            int keypointNum = 0;
            ObservationWithCandidateStates dstTimeStep = timeSteps.get(timeSteps.size()-1);
            Map<State,Double> forwardLenMap = new HashMap<>(); //the min distance from origin to state
            Map<State,SequenceState<State, Observation, Path>> forwardPathMap = new HashMap<>();
            Map<State,State> forwardPrevStateMap = new HashMap<>(); //the previous state of the min path from origin to state
            Path bestPathToPrev = null;
            Path bestPathToDst = null;
            State bestDstState = null;
            ObservationWithCandidateStates prevKeypoint = null;
            int action = 0;
            boolean isKey = false;
            double[] prevObsToPathDis = {0.0,0.0};
            double obsToPathDis = 0.0;
            List<Float> lats = new ArrayList<>();
            List<Float> lons = new ArrayList<>();
            for(int i=0;i<timeSteps.size();i++){
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
                //origin obs is keypoint
                isKey = false;
                if(i==0){
                    isKey = true;
                }
                if(i>0){
                    ObjectMapper mapper = new ObjectMapper();
                    obsToPathDis = pointToEdgesDis(timeStep.observation,allEdges);

                    long t1 = System.currentTimeMillis();
                    if(i==3||(obsToPathDis>20&&i>3)){ //if obsToPathDis<=20, there must be a candidate point on the path
                        List<Double> nextState = new ArrayList<>();
                        nextState.add(obsToPathDis);
                        nextState.add(prevObsToPathDis[0]);
                        nextState.add(prevObsToPathDis[1]);
                        double l1 = timeSteps.get(i-2).observation.getPoint().lat - timeSteps.get(i).observation.getPoint().lat;
                        double l2 = timeSteps.get(i-2).observation.getPoint().lon - timeSteps.get(i).observation.getPoint().lon;
                        double l3 = timeSteps.get(i-1).observation.getPoint().lat - timeSteps.get(i).observation.getPoint().lat;
                        double l4 = timeSteps.get(i-1).observation.getPoint().lon - timeSteps.get(i).observation.getPoint().lon;
                        nextState.add(l1*1e5); //scale lat and lon to have the same order of magnitude as obsToPathDis
                        nextState.add(l2*1e5);
                        nextState.add(l3*1e5);
                        nextState.add(l4*1e5);
                        rlInfo rlreturn = new rlInfo(nextState,0,false,lats,lons);
                        out.println(mapper.writeValueAsString(rlreturn)); // write state
                        String info = in.readLine(); // get action
                        action = parseInt(info);
                    }else{
                        action = 0;
                    }

                    long t2 = System.currentTimeMillis();
                    socketTime += t2 - t1;

                    isKey = action != 0;
                }
                //update path
                if(isKey){
                    prevObsToPathDis[0] = 0;
                    prevObsToPathDis[1] = 0;
                    keypointNum+=1;
                    ObservationWithCandidateStates keyPoint = timeSteps.get(i);
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
                            forwardPrevStateMap.put(candidate,bestLastState);
                        }
                        //calculate the min distance to dst
                        double minDisToDst = Double.MAX_VALUE;
                        Path minPathToDst = null;
                        State tempDst = null;
                        for(State dstCandidate:dstTimeStep.candidates){
                            Path path = KMM.router.calcPath(candidate.getSnap().getClosestNode(), dstCandidate.getSnap().getClosestNode(),EdgeIterator.ANY_EDGE,EdgeIterator.ANY_EDGE);
                            double pathDistance = getPathWeight(path);
                            if(pathDistance < minDisToDst){
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
                            forwardPrevStateMap.put(bestDstState,candidate);
                        }
                    }
                    prevKeypoint = keyPoint;
                }else{
                    prevObsToPathDis[1] = prevObsToPathDis[0];
                    prevObsToPathDis[0] = obsToPathDis;
                }
            }
            ObjectMapper mapper = new ObjectMapper();
            List<Double> nextState = new ArrayList<>();
            for(int i=0;i<7;i++)nextState.add(0.0);
            int reward = action == 0?0:-1;
            rlInfo rlreturn = new rlInfo(nextState,reward,true,lats,lons);
            out.println(mapper.writeValueAsString(rlreturn)); // 写入套接字
            echoSocket.close();
            result.add(new SequenceState<>(bestDstState,timeSteps.get(timeSteps.size()-1).observation,bestPathToDst));
            State thisCandidate = forwardPrevStateMap.get(bestDstState);
            while(forwardPathMap.containsKey(thisCandidate)){
                result.add(forwardPathMap.get(thisCandidate));
                thisCandidate = forwardPrevStateMap.get(thisCandidate);
            }
            Collections.reverse(result);
        }catch (UnknownHostException e) {
            System.err.println("Don't know about host " + hostName);
            System.exit(1);
        } catch (IOException e) {
            System.err.println("Couldn't get I/O for the connection to " +
                    hostName);
            System.exit(1);
        }
        System.out.print("Socket Time:"+socketTime);
        return result;
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
