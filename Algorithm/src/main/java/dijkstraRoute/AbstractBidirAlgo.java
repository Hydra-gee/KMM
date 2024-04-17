/*
 *  Licensed to GraphHopper GmbH under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for
 *  additional information regarding copyright ownership.
 *
 *  GraphHopper GmbH licenses this file to you under the Apache License,
 *  Version 2.0 (the "License"); you may not use this file except in
 *  compliance with the License. You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package dijkstraRoute;
import chRoute.PathCache;
import com.carrotsearch.hppc.IntArrayList;
import com.carrotsearch.hppc.IntObjectMap;
import com.graphhopper.coll.GHIntObjectHashMap;
import com.graphhopper.routing.EdgeToEdgeRoutingAlgorithm;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.SPTEntry;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

import static com.graphhopper.util.EdgeIterator.ANY_EDGE;

public abstract class AbstractBidirAlgo implements EdgeToEdgeRoutingAlgorithm {
    protected final TraversalMode traversalMode;
    protected int from;
    protected int to;
    protected int fromOutEdge;
    protected int toInEdge;
    protected IntObjectMap<SPTEntry> bestWeightMapFrom;
    protected IntObjectMap<SPTEntry> bestWeightMapTo;
    protected IntObjectMap<SPTEntry> bestWeightMapOther;
    protected SPTEntry currFrom;
    protected SPTEntry currTo;
    protected SPTEntry bestFwdEntry;
    protected SPTEntry bestBwdEntry;
    protected double bestWeight = Double.MAX_VALUE;
    protected int maxVisitedNodes = Integer.MAX_VALUE;
    protected long timeoutMillis = Long.MAX_VALUE;
    private long finishTimeMillis = Long.MAX_VALUE;
    PriorityQueue<SPTEntry> pqOpenSetFrom;
    PriorityQueue<SPTEntry> pqOpenSetTo;
    protected boolean updateBestPath = true;
    protected boolean finishedFrom;
    protected boolean finishedTo;
    int visitedCountFrom;
    int visitedCountTo;
    private boolean alreadyRun;
    public PathCache pathCache;
    public Graph graph;
    public Path pCache=null;
    public SPTEntry fromCache=null,toCache=null;
    
    public SPTEntry nowFrom = null,nowTo = null;
    public AbstractBidirAlgo(TraversalMode traversalMode, PathCache pathCache,Graph graph) {
        this.traversalMode = traversalMode;
        fromOutEdge = ANY_EDGE;
        toInEdge = ANY_EDGE;
        this.pathCache = pathCache;
        this.graph = graph;
    }

    protected void initCollections(int size) {
        pqOpenSetFrom = new PriorityQueue<>(size);
        bestWeightMapFrom = new GHIntObjectHashMap<>(size);

        pqOpenSetTo = new PriorityQueue<>(size);
        bestWeightMapTo = new GHIntObjectHashMap<>(size);
    }

    /**
     * Creates the root shortest path tree entry for the forward or backward search.
     */
    protected abstract SPTEntry createStartEntry(int node, double weight, boolean reverse);

    @Override
    public List<Path> calcPaths(int from, int to) {
        return Collections.singletonList(calcPath(from, to));
    }

    @Override
    public Path calcPath(int from, int to) {
        return calcPath(from, to, ANY_EDGE, ANY_EDGE);
    }

    @Override
    public Path calcPath(int from, int to, int fromOutEdge, int toInEdge) {
        if ((fromOutEdge != ANY_EDGE || toInEdge != ANY_EDGE) && !traversalMode.isEdgeBased()) {
            throw new IllegalArgumentException("Restricting the start/target edges is only possible for edge-based graph traversal");
        }
        this.fromOutEdge = fromOutEdge;
        this.toInEdge = toInEdge;
        checkAlreadyRun();
        setupFinishTime();
        pCache = null;
        if(pathCache.cacheType!=0){
            pCache = getCachedPath(from,to);
        }
        //检查是否有缓存
//        double minLen = Double.MAX_VALUE;
//        for(int j = pathCache.size()-1; j >= 0; j--){
//            Path p = pathCache.get(j);
//            if(!p.isFound()){
//                continue;
//            }
//            IntIndexedContainer nodesCache = p.calcNodes();
//            if(nodesCache.contains(from) && nodesCache.contains(to)&&nodesCache.indexOf(from)<nodesCache.indexOf(to)){
//                System.out.println("Cached a "+j);
//                Path tempP = new Path(graph);
//                tempP.setFound(false);
//                tempP.setFromNode(from);
//                tempP.setEndNode(to);
//                for(int i = nodesCache.indexOf(from); i < nodesCache.indexOf(to); i++) {
////                    pCache.addEdge(p.getEdges().get(i));
//                    tempP.addEdge(p.getEdges().get(i));
//                }
//                double len = getPathDistance(tempP);
//                if(len<minLen){
//                    minLen = len;
//                    pCache = tempP;
//                }
//            }
//        }
        if (pCache != null) {
            return pCache;
        }

        init(from, 0, to, 0);
        long t1 = System.currentTimeMillis();
        runAlgo();
        long t2 = System.currentTimeMillis();
        Path result;
        if(pCache!=null){
            result = extractPath2(pCache,fromCache,toCache);
            result.setFound(false);
//            for(int i=result.getEdges().size()-1;i>0;i--){
//                if(result.getEdges().get(i) == result.getEdges().get(i-1)){
//                    result.getEdges().remove(i);
//                }
//            }
        }else{
            result = extractPath();
        }
        long t3 = System.currentTimeMillis();
//        System.out.println("algoTime: " + (t2 - t1) + " extractTime: " + (t3 - t2));
        return result;
    }

    void init(int from, double fromWeight, int to, double toWeight) {
        initFrom(from, fromWeight);
        initTo(to, toWeight);
        postInit(from, to);
    }

    protected void initFrom(int from, double weight) {
        this.from = from;
        currFrom = createStartEntry(from, weight, false);
        pqOpenSetFrom.add(currFrom);
        if (!traversalMode.isEdgeBased()) {
            bestWeightMapFrom.put(from, currFrom);
        }
    }

    protected void initTo(int to, double weight) {
        this.to = to;
        currTo = createStartEntry(to, weight, true);
        pqOpenSetTo.add(currTo);
        if (!traversalMode.isEdgeBased()) {
            bestWeightMapTo.put(to, currTo);
        }
    }

    protected void postInit(int from, int to) {
        if (!traversalMode.isEdgeBased()) {
            if (updateBestPath) {
                bestWeightMapOther = bestWeightMapFrom;
                updateBestPath(Double.POSITIVE_INFINITY, currFrom, EdgeIterator.NO_EDGE, to, true);
            }
        } else if (from == to && fromOutEdge == ANY_EDGE && toInEdge == ANY_EDGE) {
            // special handling if start and end are the same and no directions are restricted
            // the resulting weight should be zero
            if (currFrom.weight != 0 || currTo.weight != 0) {
                throw new IllegalStateException("If from=to, the starting weight must be zero for from and to");
            }
            bestFwdEntry = currFrom;
            bestBwdEntry = currTo;
            bestWeight = 0;
            finishedFrom = true;
            finishedTo = true;
            return;
        }
        postInitFrom();
        postInitTo();
    }

    protected abstract void postInitFrom();

    protected abstract void postInitTo();
    protected void runAlgo() {
        while (!finished() && !isMaxVisitedNodesExceeded() && !isTimeoutExceeded()) {
            if (!finishedFrom) {
                nowFrom = fillEdgesFrom();
                finishedFrom = nowFrom ==null;
            }
            double time1 = System.currentTimeMillis();
            pCache = null;
            if(nowFrom!=null&&nowTo!=null&&pathCache.cacheType==2){
                pCache = getCachedPath(nowFrom.adjNode,nowTo.adjNode);
                fromCache = nowFrom;
                toCache = nowTo;
            }
            if(pCache!=null){
                return;
            }
            if (!finishedTo) {
                nowTo = fillEdgesTo();
//                nowTo = nowto;
                finishedTo = nowTo ==null;
            }
            double time3 = System.currentTimeMillis();
            pCache = null;
            if(nowFrom!=null&&nowTo!=null&&pathCache.cacheType==2){
                pCache = getCachedPath(nowFrom.adjNode,nowTo.adjNode);
                fromCache = nowFrom;
                toCache = nowTo;
            }
            if(pCache!=null){
                return;
            }
        }
    }
//    protected void runAlgo() {
//        while (!finished() && !isMaxVisitedNodesExceeded() && !isTimeoutExceeded()) {
////            SPTEntry nowFrom=null,nowTo=null;
//            if (!finishedFrom) {
//                nowFrom = fillEdgesFrom();
////                nowFrom = nowfrom;
//                finishedFrom = nowFrom ==null;
//            }
////            if(nowFrom!=null && nowTo!=null){
////                double minLen = Double.MAX_VALUE;
////                for(int j = pathCache.size()-1; j >= 0; j--){
////                    Path p = pathCache.get(j);
////                    if(!p.isFound()){
////                        continue;
////                    }
////                    IntIndexedContainer nodesCache = p.calcNodes();
////                    if(nodesCache.contains(nowFrom.adjNode) && nodesCache.contains(nowTo.adjNode) && nodesCache.indexOf(nowFrom.adjNode)<nodesCache.indexOf(nowTo.adjNode)){
////                        System.out.println("Cached b "+j + " "+nowFrom.adjNode+" "+nowTo.adjNode);
////                        Path tempP = new Path(graph);
//////                        pCache = new Path(graph);
////                        this.fromCache = nowFrom;
////                        this.toCache = nowTo;
////                        for(int i = nodesCache.indexOf(nowFrom.adjNode); i < nodesCache.indexOf(nowTo.adjNode); i++) {
////                            tempP.addEdge(p.getEdges().get(i));
////                        }
////                        tempP.setFromNode(nowFrom.adjNode);
////                        tempP.setEndNode(nowTo.adjNode);
////                        double len = getPathDistance(tempP);
////                        if(len<minLen){
////                            minLen = len;
////                            pCache = tempP;
////                        }
////                    }
////                }
////            }
//            if(pCache!=null){
//                return;
//            }
//            if (!finishedTo) {
//                nowTo = fillEdgesTo();
////                nowTo = nowto;
//                finishedTo = nowTo ==null;
//            }
//            if(nowFrom!=null && nowTo!=null){
//                double minLen = Double.MAX_VALUE;
//                for(int j = pathCache.size()-1; j >= 0; j--){
//                    Path p = pathCache.get(j);
//                    if(!p.isFound()){
//                        continue;
//                    }
//                    IntIndexedContainer nodesCache = p.calcNodes();
//                    if(nodesCache.contains(nowFrom.adjNode) && nodesCache.contains(nowTo.adjNode) && nodesCache.indexOf(nowFrom.adjNode)<nodesCache.indexOf(nowTo.adjNode)){
//                        System.out.println("Cached b "+j + " "+nowFrom.adjNode+" "+nowTo.adjNode);
//                        Path tempP = new Path(graph);
////                        pCache = new Path(graph);
//                        this.fromCache = nowFrom;
//                        this.toCache = nowTo;
//                        for(int i = nodesCache.indexOf(nowFrom.adjNode); i < nodesCache.indexOf(nowTo.adjNode); i++) {
//                            tempP.addEdge(p.getEdges().get(i));
//                        }
//                        tempP.setFromNode(nowFrom.adjNode);
//                        tempP.setEndNode(nowTo.adjNode);
//                        double len = getPathDistance(tempP);
//                        if(len<minLen){
//                            minLen = len;
//                            pCache = tempP;
//                        }
//                    }
//                }
//            }
//            if(pCache!=null){
//                return;
//            }
//        }
//    }

    // http://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/EPP%20shortest%20path%20algorithms.pdf
    // a node from overlap may not be on the best path!
    // => when scanning an arc (v, w) in the forward search and w is scanned in the reverseOrder
    //    search, update extractPath = μ if df (v) + (v, w) + dr (w) < μ
    protected boolean finished() {
        if (finishedFrom || finishedTo)
            return true;

        return currFrom.weight + currTo.weight >= bestWeight;
    }

    abstract SPTEntry fillEdgesFrom();

    abstract SPTEntry fillEdgesTo();

    protected void updateBestPath(double edgeWeight, SPTEntry entry, int origEdgeIdForCH, int traversalId, boolean reverse) {
        assert traversalMode.isEdgeBased() != Double.isInfinite(edgeWeight);
        SPTEntry entryOther = bestWeightMapOther.get(traversalId);
        if (entryOther == null)
            return;

        // update μ
        double weight = entry.getWeightOfVisitedPath() + entryOther.getWeightOfVisitedPath();
        if (traversalMode.isEdgeBased()) {
            if (getIncomingEdge(entryOther) != getIncomingEdge(entry))
                throw new IllegalStateException("cannot happen for edge based execution of " + getName());

            // prevents the path to contain the edge at the meeting point twice and subtracts the weight (excluding turn weight => no previous edge)
            entry = entry.getParent();
            weight -= edgeWeight;
        }

        if (weight < bestWeight) {
            bestFwdEntry = reverse ? entryOther : entry;
            bestBwdEntry = reverse ? entry : entryOther;
            bestWeight = weight;
        }
    }

    protected abstract double getInEdgeWeight(SPTEntry entry);

    protected int getIncomingEdge(SPTEntry entry) {
        return entry.edge;
    }

    abstract protected Path extractPath();
    abstract protected Path extractPath2(Path pCache,SPTEntry fromCache,SPTEntry toCache);
    protected boolean fromEntryCanBeSkipped() {
        return false;
    }

    protected boolean fwdSearchCanBeStopped() {
        return false;
    }

    protected boolean toEntryCanBeSkipped() {
        return false;
    }

    protected boolean bwdSearchCanBeStopped() {
        return false;
    }

    protected double getCurrentFromWeight() {
        return currFrom.weight;
    }

    protected double getCurrentToWeight() {
        return currTo.weight;
    }

    IntObjectMap<SPTEntry> getBestFromMap() {
        return bestWeightMapFrom;
    }

    IntObjectMap<SPTEntry> getBestToMap() {
        return bestWeightMapTo;
    }

    void setBestOtherMap(IntObjectMap<SPTEntry> other) {
        bestWeightMapOther = other;
    }

    protected void setUpdateBestPath(boolean b) {
        updateBestPath = b;
    }

    @Override
    public int getVisitedNodes() {
        return visitedCountFrom + visitedCountTo;
    }

    void setToDataStructures(AbstractBidirAlgo other) {
        to = other.to;
        toInEdge = other.toInEdge;
        pqOpenSetTo = other.pqOpenSetTo;
        bestWeightMapTo = other.bestWeightMapTo;
        finishedTo = other.finishedTo;
        currTo = other.currTo;
        visitedCountTo = other.visitedCountTo;
        // inEdgeExplorer
    }

    @Override
    public void setMaxVisitedNodes(int numberOfNodes) {
        this.maxVisitedNodes = numberOfNodes;
    }

    @Override
    public void setTimeoutMillis(long timeoutMillis) {
        this.timeoutMillis = timeoutMillis;
    }

    protected void checkAlreadyRun() {
        if (alreadyRun)
            throw new IllegalStateException("Create a new instance per call");

        alreadyRun = true;
    }

    protected void setupFinishTime() {
        try {
            this.finishTimeMillis = Math.addExact(System.currentTimeMillis(), timeoutMillis);
        } catch (ArithmeticException e) {
            this.finishTimeMillis = Long.MAX_VALUE;
        }
    }

    @Override
    public String getName() {
        return getClass().getSimpleName();
    }

    protected boolean isMaxVisitedNodesExceeded() {
        return maxVisitedNodes < getVisitedNodes();
    }

    protected boolean isTimeoutExceeded() {
        return finishTimeMillis < Long.MAX_VALUE && System.currentTimeMillis() > finishTimeMillis;
    }

    public double getPathDistance(Path path){
        double result = 0;
        for(EdgeIteratorState e:path.calcEdges()){
            result += e.getDistance();
        }
        return result;
    }

    public Path getCachedPath(int from,int to){
        double minLen = Double.MAX_VALUE;
        Path result = null;
        if(pathCache.pathIndexMap.containsKey(from)&&pathCache.pathIndexMap.containsKey(to)){
            List<Integer> intersection = new ArrayList<>(pathCache.pathIndexMap.get(from));
            intersection.retainAll(pathCache.pathIndexMap.get(to));
            if(!intersection.isEmpty()){
                for(int i :intersection){
                    if(pathCache.posIndexMap.get(from).get(i)>=pathCache.posIndexMap.get(to).get(i)){
                        continue;
                    }
                    Path p = pathCache.pathList.get(i);
                    if(p.isFound()){
                        Path tempP = new Path(graph);
                        tempP.setFound(false);
                        tempP.setFromNode(from);
                        tempP.setEndNode(to);
                        IntArrayList edges = p.getEdges();
                        for(int j = pathCache.posIndexMap.get(from).get(i); j < pathCache.posIndexMap.get(to).get(i); j++) {
                            tempP.addEdge(edges.get(j));
                        }
                        double len = getPathDistance(tempP);
                        if(len<minLen){
                            minLen = len;
                            result = tempP;
                        }
                    }
                }
            }
        }
        return result;
    }
}
