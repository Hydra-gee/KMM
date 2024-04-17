package matchAlgo;
import com.graphhopper.routing.Path;
import com.graphhopper.util.EdgeIteratorState;
import entities.Observation;
import entities.ObservationWithCandidateStates;
import entities.SequenceState;
import entities.State;
import kmm.KMM;

import java.util.List;

import static java.lang.Math.max;
import static java.lang.Math.min;

public abstract class BaseMatch {
    protected KMM KMM = null;
    public BaseMatch(KMM KMM){
        this.KMM = KMM;
    }
    public abstract List<SequenceState<State, Observation, Path>>  match(List<ObservationWithCandidateStates> timeSteps);

    double calcShrinkFactor(double a_lat_deg, double b_lat_deg) {
        return Math.cos(Math.toRadians((a_lat_deg + b_lat_deg) / 2.0));
    }

    public double calcNormalizedDist(double fromLat, double fromLon, double toLat, double toLon) {
        double sinDeltaLat = Math.sin(Math.toRadians(toLat - fromLat) / 2.0);
        double sinDeltaLon = Math.sin(Math.toRadians(toLon - fromLon) / 2.0);
        return sinDeltaLat * sinDeltaLat + sinDeltaLon * sinDeltaLon * Math.cos(Math.toRadians(fromLat)) * Math.cos(Math.toRadians(toLat));
    }

    //保证垂足在线段上
    public double calcNormalizedEdgeDistance(double r_lat_deg, double r_lon_deg, double a_lat_deg, double a_lon_deg, double b_lat_deg, double b_lon_deg) {
        double shrinkFactor = this.calcShrinkFactor(a_lat_deg, b_lat_deg);
        double a_lon = a_lon_deg * shrinkFactor;
        double b_lon = b_lon_deg * shrinkFactor;
        double r_lon = r_lon_deg * shrinkFactor;
        double delta_lon = b_lon - a_lon;
        double delta_lat = b_lat_deg - a_lat_deg;
        if (delta_lat == 0.0) {
            return this.calcNormalizedDist(a_lat_deg, r_lon_deg, r_lat_deg, r_lon_deg);
        } else if (delta_lon == 0.0) {
            return this.calcNormalizedDist(r_lat_deg, a_lon_deg, r_lat_deg, r_lon_deg);
        } else {
            double norm = delta_lon * delta_lon + delta_lat * delta_lat;
            double factor = ((r_lon - a_lon) * delta_lon + (r_lat_deg - a_lat_deg) * delta_lat) / norm;
            double c_lon = a_lon + factor * delta_lon;
            double c_lat = a_lat_deg + factor * delta_lat;
            double lat_min = min(a_lat_deg,b_lat_deg);
            double lat_max = max(a_lat_deg,b_lat_deg);
            double lon_min = min(a_lon_deg,b_lon_deg);
            double lon_max = max(a_lon_deg,b_lon_deg);
            if(c_lat>lat_min&&c_lat<lat_max&&c_lon / shrinkFactor>lon_min&&c_lon / shrinkFactor<lon_max){
                return this.calcNormalizedDist(c_lat, c_lon / shrinkFactor, r_lat_deg, r_lon_deg);
            }else{
                return min(this.calcNormalizedDist(a_lat_deg, a_lon_deg, r_lat_deg, r_lon_deg),
                        this.calcNormalizedDist(b_lat_deg, b_lon_deg, r_lat_deg, r_lon_deg));
            }
        }
    }

    public double pointToEdgesDis(Observation obs,List<EdgeIteratorState> edges){
        double minDis = Double.MAX_VALUE;
        for(EdgeIteratorState e:edges){
            double n1lat = KMM.queryGraph.getNodeAccess().getLat(e.getBaseNode());
            double n1lon = KMM.queryGraph.getNodeAccess().getLon(e.getBaseNode());
            double n2lat = KMM.queryGraph.getNodeAccess().getLat(e.getAdjNode());
            double n2lon = KMM.queryGraph.getNodeAccess().getLon(e.getAdjNode());
            double dis = calcNormalizedEdgeDistance(obs.getPoint().lat, obs.getPoint().lon,n1lat,n1lon,n2lat,n2lon);
            dis = KMM.distanceCalc.calcDenormalizedDist(dis);
            minDis = min(minDis,dis);
        }
        return minDis;
    }

}
