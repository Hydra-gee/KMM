package preprocessAlgo;


import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.storage.index.Snap;
import entities.Observation;
import entities.ObservationWithCandidateStates;
import kmm.KMM;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class downSamplePreProcess extends BasePreProcess{
    public downSamplePreProcess(KMM KMM) {
        super(KMM);
    }

    @Override
    public List<ObservationWithCandidateStates> preprocess(List<Observation> observations) {
        //downsample observations
        List<Observation> filteredObservations = filterObservations(observations);
        List<Collection<Snap>> splitsPerObservation = filteredObservations.stream().map(o -> findCandidateSnaps(o.getPoint().getLat(),o.getPoint().getLon()).
                        stream().filter(s-> s.getQueryDistance() <= KMM.mmConfig.getMeasurementErrorSigma()).collect(Collectors.toList()))
                .collect(Collectors.toList());
        KMM.queryGraph = QueryGraph.create(KMM.graph, splitsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));
        splitsPerObservation = splitsPerObservation.stream().map(this::deduplicate).collect(Collectors.toList());
        return createTimeSteps(filteredObservations, splitsPerObservation).stream().filter(t-> !t.candidates.isEmpty()).collect(Collectors.toList());
    }
}
