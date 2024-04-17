package kmm;

import chRoute.CHRoutingAlgorithmFactory;
import chRoute.PathCache;
import com.carrotsearch.hppc.IntIndexedContainer;
import com.graphhopper.GraphHopper;
import com.graphhopper.config.Profile;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.RoutingAlgorithm;
import com.graphhopper.routing.ev.BooleanEncodedValue;
import com.graphhopper.routing.ev.Subnetwork;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.querygraph.QueryRoutingCHGraph;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.DefaultSnapFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.*;
import com.graphhopper.storage.index.LocationIndexTree;
import com.graphhopper.util.*;
import entities.*;
import matchAlgo.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import preprocessAlgo.downSamplePreProcess;
import dijkstraRoute.CacheTree;
import dijkstraRoute.DijkstraBidirectiondiy;

import java.util.*;
import java.util.stream.Collectors;

import static com.graphhopper.util.Parameters.Algorithms.ASTAR_BI;
import static com.graphhopper.util.Parameters.Routing.ALGORITHM;

public class KMM {
    public final Logger logger = LoggerFactory.getLogger(getClass());
    public final GraphHopper hopper;
    public final BaseGraph graph;
    public final Router router;
    public final LocationIndexTree locationIndex;
    public final DistanceCalc distanceCalc = new DistancePlaneProjection();
    public final Weighting weighting;
    public QueryGraph queryGraph;
    public RoutingCHGraph chGraph;
    public Config mmConfig;
    public RoutingCHGraph chGraph2 = null;
    public CacheTree cacheTree = null;
    public List<Path> pathCache = null;
    public PathCache pathCache2 = null;
    public int keypointNums = 0;
    public List<Integer> keypoints = new ArrayList<>();

    public KMM(GraphHopper graphHopper, PMap hints, RoutingCHGraph chGraph, Config config) {
        this.hopper = graphHopper;
        this.locationIndex = (LocationIndexTree) graphHopper.getLocationIndex();
        String profileStr = hints.getString("profile", "");
        Profile profile = graphHopper.getProfile(profileStr);
        graph = graphHopper.getBaseGraph();
        weighting = graphHopper.createWeighting(profile, hints);
        this.chGraph = chGraph;
        router = createRouter();
        mmConfig = config;
    }
    public MatchResult match(List<Observation> observations) {
        BaseMatch[] matchAlgorithms = {
                new SPMatch(this), //shortest path
                new KPMatch(this), // rule-based keypoint selection
                new RLMatch(this), // reinforcement learning based keypoint selection
                new STMatch(this), // the reimplementation version of the ST-Matching algorithm
        };

        int index = this.mmConfig.getAlgorithm();
        if (index < 0 || index >= matchAlgorithms.length) {
            index = 1; // default algorithm is rule-based keypoint selection
        }
        BaseMatch matchAlgo = matchAlgorithms[index];
        downSamplePreProcess preprocessAlgo = new downSamplePreProcess(this);
        List<ObservationWithCandidateStates> timeSteps = preprocessAlgo.preprocess(observations);
        List<SequenceState<State, Observation, Path>> seq = matchAlgo.match(timeSteps);
        //generate Result
        List<EdgeIteratorState> path = seq.stream().filter(s1 -> s1.transitionDescriptor != null).flatMap(s1 -> s1.transitionDescriptor.calcEdges().stream()).collect(Collectors.toList());
        MatchResult result = new MatchResult(prepareEdgeMatches(seq));
        //设置观测点和候选点坐标
        List<PointCoordinate> pointCoordinates = new ArrayList<>();
        for(SequenceState<State, Observation, Path> s:seq){
            pointCoordinates.add(new PointCoordinate(s.observation.getPoint().lat, s.observation.getPoint().lon,
                    s.state.getSnap().getSnappedPoint().getLat(),s.state.getSnap().getSnappedPoint().getLon()));
        }
        result.setPointCoordinates(pointCoordinates);
        result.setMergedPath(new MapMatchedPath(queryGraph, weighting, path));
        result.setMatchMillis(seq.stream().filter(s -> s.transitionDescriptor != null).mapToLong(s -> s.transitionDescriptor.getTime()).sum());
        result.setMatchLength(seq.stream().filter(s -> s.transitionDescriptor != null).mapToDouble(s -> s.transitionDescriptor.getDistance()).sum());
        result.setGPXEntriesLength(gpxLength(observations));
        result.setGraph(queryGraph);
        result.setWeighting(weighting);
        result.keypointNums = keypointNums;
        result.keypointIndex = keypoints;
        return result;
    }

    private List<EdgeMatch> prepareEdgeMatches(List<SequenceState<State, Observation, Path>> seq) {
        List<EdgeMatch> edgeMatches = new ArrayList<>();
        List<State> states = new ArrayList<>();
        EdgeIteratorState currentDirectedRealEdge = null;
        for (SequenceState<State, Observation, Path> transitionAndState : seq) {
            // transition (except before the first state)
            if (transitionAndState.transitionDescriptor != null) {
                for (EdgeIteratorState edge : transitionAndState.transitionDescriptor.calcEdges()) {
                    EdgeIteratorState newDirectedRealEdge = resolveToRealEdge(edge);
                    if (currentDirectedRealEdge != null) {
                        if (!equalEdges(currentDirectedRealEdge, newDirectedRealEdge)) {
                            EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
                            edgeMatches.add(edgeMatch);
                            states = new ArrayList<>();
                        }
                    }
                    currentDirectedRealEdge = newDirectedRealEdge;
                }
            }
            // state
            if (transitionAndState.state.isOnDirectedEdge()) { // as opposed to on a node
                EdgeIteratorState newDirectedRealEdge = resolveToRealEdge(transitionAndState.state.getOutgoingVirtualEdge());
                if (currentDirectedRealEdge != null) {
                    if (!equalEdges(currentDirectedRealEdge, newDirectedRealEdge)) {
                        EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
                        edgeMatches.add(edgeMatch);
                        states = new ArrayList<>();
                    }
                }
                currentDirectedRealEdge = newDirectedRealEdge;
            }
            states.add(transitionAndState.state);
        }
        if (currentDirectedRealEdge != null) {
            EdgeMatch edgeMatch = new EdgeMatch(currentDirectedRealEdge, states);
            edgeMatches.add(edgeMatch);
        }
        return edgeMatches;
    }

    private double gpxLength(List<Observation> gpxList) {
        if (gpxList.isEmpty()) {
            return 0;
        } else {
            double gpxLength = 0;
            Observation prevEntry = gpxList.get(0);
            for (int i = 1; i < gpxList.size(); i++) {
                Observation entry = gpxList.get(i);
                gpxLength += distanceCalc.calcDist(prevEntry.getPoint().lat, prevEntry.getPoint().lon, entry.getPoint().lat, entry.getPoint().lon);
                prevEntry = entry;
            }
            return gpxLength;
        }
    }

    private boolean equalEdges(EdgeIteratorState edge1, EdgeIteratorState edge2) {
        return edge1.getEdge() == edge2.getEdge()
                && edge1.getBaseNode() == edge2.getBaseNode()
                && edge1.getAdjNode() == edge2.getAdjNode();
    }

    private EdgeIteratorState resolveToRealEdge(EdgeIteratorState edgeIteratorState) {
        if (queryGraph.isVirtualNode(edgeIteratorState.getBaseNode()) || queryGraph.isVirtualNode(edgeIteratorState.getAdjNode())) {
            return graph.getEdgeIteratorStateForKey(((VirtualEdgeIteratorState) edgeIteratorState).getOriginalEdgeKey());
        } else {
            return edgeIteratorState;
        }
    }

    public String getSnappedCandidates(Collection<State> candidates) {
        String str = "";
        for (State gpxe : candidates) {
            if (!str.isEmpty()) {
                str += ", ";
            }
            str += "distance: " + gpxe.getSnap().getQueryDistance() + " to "
                    + gpxe.getSnap().getSnappedPoint();
        }
        return "[" + str + "]";
    }

    protected Router createRouter() {
        BooleanEncodedValue inSubnetworkEnc = hopper.getEncodingManager().getBooleanEncodedValue(Subnetwork.key("car"));
        DefaultSnapFilter snapFilter = new DefaultSnapFilter(weighting, inSubnetworkEnc);

        Router router = new Router() {
            @Override
            public EdgeFilter getSnapFilter() {
                return snapFilter;
            }

            @Override
            public Path calcPath(int fromNode,int toNode,int fromOutEdge,int toInEdge) {
                if(chGraph2==null){
                    chGraph2 = new QueryRoutingCHGraph(chGraph,queryGraph);
                }
                if(pathCache==null){
                    pathCache = new ArrayList<>();
                }
                if(pathCache2==null){
                    pathCache2 = new PathCache();
                    pathCache2.cacheType = mmConfig.getCacheType();
                }
                if(mmConfig.getPathCalculateAlgo()==1){
                    long t1 = System.currentTimeMillis();
                    Weighting queryGraphWeighting = queryGraph.wrapWeighting(weighting);
                    if(cacheTree==null){
                        cacheTree = new CacheTree();
                    }
                    DijkstraBidirectiondiy dijkstraBidirectiondiy = new DijkstraBidirectiondiy(queryGraph, queryGraphWeighting, TraversalMode.NODE_BASED,pathCache2) {
                        @Override
                        protected void initCollections(int size) {
                            super.initCollections(50);
                        }
                    };
                    dijkstraBidirectiondiy.setMaxVisitedNodes(Integer.MAX_VALUE);
                    Path result = dijkstraBidirectiondiy.calcPath(fromNode, toNode, fromOutEdge, toInEdge);
                    if(result.isFound()){
                        pathCache2.pathList.add(result);
                        IntIndexedContainer nodes = result.calcNodes();
                        for(int i = 0; i < nodes.size(); i++){
                            pathCache2.pathIndexMap.computeIfAbsent(nodes.get(i), k -> new ArrayList<>());
                            pathCache2.posIndexMap.computeIfAbsent(nodes.get(i), k -> new HashMap<>());
                            pathCache2.pathIndexMap.get(nodes.get(i)).add(pathCache2.pathList.size()-1);
                            pathCache2.posIndexMap.get(nodes.get(i)).put(pathCache2.pathList.size()-1,i);
                        }
                    }
                    long t2 = System.currentTimeMillis();
                    return result;
                }else{
                    long t1 = System.currentTimeMillis();
                    Path result = null;
                    result = createCHAlgo(chGraph2, true).calcPath(fromNode, toNode);
                    if(result.isFound()){
                        pathCache2.pathList.add(result);
                        IntIndexedContainer nodes = result.calcNodes();
                        for(int i = 0; i < nodes.size(); i++){
                            pathCache2.pathIndexMap.computeIfAbsent(nodes.get(i), k -> new ArrayList<>());
                            pathCache2.posIndexMap.computeIfAbsent(nodes.get(i), k -> new HashMap<>());
                            pathCache2.pathIndexMap.get(nodes.get(i)).add(pathCache2.pathList.size()-1);
                            pathCache2.posIndexMap.get(nodes.get(i)).put(pathCache2.pathList.size()-1,i);
                        }
                    }
                    long t2 = System.currentTimeMillis();
                    return result;
                }
            }

            @Override
            public Weighting getWeighting() {
                return weighting;
            }
        };
        return router;
    }

    public static class MapMatchedPath extends Path {
        public MapMatchedPath(Graph graph, Weighting weighting, List<EdgeIteratorState> edges) {
            super(graph);
            int prevEdge = EdgeIterator.NO_EDGE;
            for (EdgeIteratorState edge : edges) {
                addDistance(edge.getDistance());
                addTime(GHUtility.calcMillisWithTurnMillis(weighting, edge, false, prevEdge));
                addEdge(edge.getEdge());
                prevEdge = edge.getEdge();
            }
            if (edges.isEmpty()) {
                setFound(false);
            } else {
                setFromNode(edges.get(0).getBaseNode());
                setFound(true);
            }
        }
    }

    public interface Router {
        EdgeFilter getSnapFilter();

        Path calcPath(int fromNode,  int toNode, int fromOutEdge,int toInEdge);

        Weighting getWeighting();

        default long getVisitedNodes() {
            return 0L;
        }
    }

    public RoutingAlgorithm createCHAlgo(RoutingCHGraph chGraph, boolean withSOD) {
        PMap opts = new PMap();
        if (!withSOD) {
            opts.putObject("stall_on_demand", false);
        }
        opts.putObject(ALGORITHM, ASTAR_BI);
        return new CHRoutingAlgorithmFactory(chGraph).createAlgo(opts,pathCache2);
    }
}