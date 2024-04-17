package preprocessAlgo;

import com.carrotsearch.hppc.IntHashSet;
import com.graphhopper.routing.querygraph.VirtualEdgeIteratorState;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.storage.index.Snap;
import com.graphhopper.util.DistanceCalcEarth;
import com.graphhopper.util.EdgeIterator;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.shapes.BBox;
import entities.Observation;
import entities.ObservationWithCandidateStates;
import entities.State;
import org.locationtech.jts.geom.Envelope;
import kmm.KMM;

import java.util.*;
import java.util.stream.Collectors;

import static com.graphhopper.util.DistancePlaneProjection.DIST_PLANE;

public abstract class BasePreProcess {
    protected KMM KMM = null;
    public BasePreProcess(KMM KMM){
        this.KMM = KMM;
    }

    public abstract List<ObservationWithCandidateStates> preprocess(List<Observation> observations);

    public Collection<Snap> deduplicate(Collection<Snap> splits) {
        // Only keep one split per node number. Let's say the last one.
        Map<Integer, Snap> splitsByNodeNumber = splits.stream().collect(Collectors.toMap(Snap::getClosestNode, s -> s, (s1, s2) -> s2));
        return splitsByNodeNumber.values();
    }

    protected List<Snap> findCandidateSnaps(final double queryLat, final double queryLon) {
        double rLon = (KMM.mmConfig.getMeasurementErrorSigma() * 360.0 / DistanceCalcEarth.DIST_EARTH.calcCircumference(queryLat));
        double rLat = KMM.mmConfig.getMeasurementErrorSigma() / DistanceCalcEarth.METERS_PER_DEGREE;
        Envelope envelope = new Envelope(queryLon, queryLon, queryLat, queryLat);
        for (int i = 0; i < 50; i++) {
            envelope.expandBy(rLon, rLat);
            List<Snap> snaps = findCandidateSnapsInBBox(queryLat, queryLon, BBox.fromEnvelope(envelope));
            if (!snaps.isEmpty()) {
                return snaps;
            }
        }
        return Collections.emptyList();
    }

    protected List<Snap> findCandidateSnapsInBBox(double queryLat, double queryLon, BBox queryShape) {
        EdgeFilter edgeFilter = KMM.router.getSnapFilter();
        List<Snap> snaps = new ArrayList<>();
        IntHashSet seenEdges = new IntHashSet();
        IntHashSet seenNodes = new IntHashSet();
        KMM.locationIndex.query(queryShape, edgeId -> {
            EdgeIteratorState edge = KMM.graph.getEdgeIteratorStateForKey(edgeId * 2);
            if (seenEdges.add(edgeId) && edgeFilter.accept(edge)) {
                Snap snap = new Snap(queryLat, queryLon);
                KMM.locationIndex.traverseEdge(queryLat, queryLon, edge, (node, normedDist, wayIndex, pos) -> {
                    if (normedDist < snap.getQueryDistance()) {
                        snap.setQueryDistance(normedDist);
                        snap.setClosestNode(node);
                        snap.setWayIndex(wayIndex);
                        snap.setSnappedPosition(pos);
                    }
                });
                double dist = DIST_PLANE.calcDenormalizedDist(snap.getQueryDistance());
                snap.setClosestEdge(edge);
                snap.setQueryDistance(dist);
                if (snap.isValid() && (snap.getSnappedPosition() != Snap.Position.TOWER || seenNodes.add(snap.getClosestNode()))) {
                    snap.calcSnappedPoint(DistanceCalcEarth.DIST_EARTH);
                    if (queryShape.contains(snap.getSnappedPoint().lat, snap.getSnappedPoint().lon)) {
                        snaps.add(snap);
                    }
                }
            }
        });
        snaps.sort(Comparator.comparingDouble(Snap::getQueryDistance));
        return snaps;
    }
    /**
     * Filters observations to only those which will be used for map matching (i.e. those which
     * are separated by at least 2 * measurementErrorSigma
     */
    protected List<Observation> filterObservations(List<Observation> observations){
        List<Observation> filtered = new ArrayList<>();
        Observation prevEntry = null;
        int last = observations.size() - 1;
        for (int i = 0; i <= last; i++) {
            Observation observation = observations.get(i);
            if (i == 0 || i == last || KMM.distanceCalc.calcDist(
                    prevEntry.getPoint().getLat(), prevEntry.getPoint().getLon(),
                    observation.getPoint().getLat(), observation.getPoint().getLon()) > KMM.mmConfig.getFilterThreshold()) {
                filtered.add(observation);
                prevEntry = observation;
            } else {
                KMM.logger.debug("Filter out observation: {}", i + 1);
            }
        }
        return filtered;
    }

    /**
     * Creates TimeSteps with candidates for the GPX entries but does not create emission or
     * transition probabilities. Creates directed candidates for virtual nodes and undirected
     * candidates for real nodes.
     */
    protected List<ObservationWithCandidateStates> createTimeSteps(List<Observation> filteredObservations, List<Collection<Snap>> splitsPerObservation) {
        if (splitsPerObservation.size() != filteredObservations.size()) {
            throw new IllegalArgumentException(
                    "filteredGPXEntries and queriesPerEntry must have same size.");
        }

        final List<ObservationWithCandidateStates> timeSteps = new ArrayList<>();
        for (int i = 0; i < filteredObservations.size(); i++) {
            Observation observation = filteredObservations.get(i);
            Collection<Snap> splits = splitsPerObservation.get(i);
            List<State> candidates = new ArrayList<>();
            for (Snap split : splits) {
                if (KMM.queryGraph.isVirtualNode(split.getClosestNode())) {
                    List<VirtualEdgeIteratorState> virtualEdges = new ArrayList<>();
                    EdgeIterator iter = KMM.queryGraph.createEdgeExplorer().setBaseNode(split.getClosestNode());
                    while (iter.next()) {
                        if (!KMM.queryGraph.isVirtualEdge(iter.getEdge())) {
                            throw new RuntimeException("Virtual nodes must only have virtual edges "
                                    + "to adjacent nodes.");
                        }
                        virtualEdges.add((VirtualEdgeIteratorState) KMM.queryGraph.getEdgeIteratorState(iter.getEdge(), iter.getAdjNode()));
                    }
                    if (virtualEdges.size() != 2) {
                        throw new RuntimeException("Each virtual node must have exactly 2 "
                                + "virtual edges (reverse virtual edges are not returned by the "
                                + "EdgeIterator");
                    }

                    // Create a directed candidate for each of the two possible directions through
                    // the virtual node. We need to add candidates for both directions because
                    // we don't know yet which is the correct one. This will be figured
                    // out by the Viterbi algorithm.
                    candidates.add(new State(observation, split, virtualEdges.get(0), virtualEdges.get(1)));
                    candidates.add(new State(observation, split, virtualEdges.get(1), virtualEdges.get(0)));
                } else {
                    // Create an undirected candidate for the real node.
                    candidates.add(new State(observation, split));
                }
            }

            timeSteps.add(new ObservationWithCandidateStates(observation, candidates));
        }
        return timeSteps;
    }
}
