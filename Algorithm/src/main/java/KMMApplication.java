import com.graphhopper.GraphHopper;
import com.graphhopper.GraphHopperConfig;
import com.graphhopper.ResponsePath;
import com.fasterxml.jackson.dataformat.xml.XmlMapper;
import com.graphhopper.routing.ch.PrepareContractionHierarchies;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.BaseGraph;
import com.graphhopper.storage.CHConfig;
import com.graphhopper.storage.RoutingCHGraph;
import com.graphhopper.storage.RoutingCHGraphImpl;
import com.graphhopper.util.*;
import entities.Config;
import entities.MatchResult;
import entities.Observation;
import entities.Gpx;
import kmm.KMM;
import spark.Request;
import spark.Response;
import com.graphhopper.config.Profile;
import spark.Spark;
import com.google.gson.Gson;

import java.io.IOException;
import java.util.*;

public class KMMApplication {
    public static GraphHopper hopper = null;
    public static RoutingCHGraph chGraph = null;
    public static String matchingRoute(Request request, Response response) throws IOException {
        //get Parameters
        String profile = request.queryParams("profile");
        boolean useVelocity = Boolean.parseBoolean(request.queryParams("useVelocity"));
        int velocityThreshold = Integer.parseInt(request.queryParams("velocityThreshold"));
        int maxKeypointNum = Integer.parseInt(request.queryParams("keypoint_num"));
        int pathCalculateAlgo = Integer.parseInt(request.queryParams("path_calc_algo"));
        int cacheType = Integer.parseInt(request.queryParams("path_cache_type"));
        int algorithm = Integer.parseInt(request.queryParams("algorithm"));
        double kpDisThreshold = Double.parseDouble(request.queryParams("kp_dis_threshold"));
        int kpConnumThreshold = Integer.parseInt(request.queryParams("kp_connum_threshold"));
        int weightType = Integer.parseInt(request.queryParams("weight_type"));
        Config config = new Config(20.0,useVelocity,velocityThreshold,40.0,maxKeypointNum,pathCalculateAlgo,cacheType,algorithm,
                kpDisThreshold,kpConnumThreshold,weightType);
        //read gpx file
        byte[] fileBytes = request.body().getBytes();
        Gpx gpx = new XmlMapper().readValue(fileBytes, Gpx.class);
        List<Observation> observations = Observation.getObservationsFromTrk(gpx.trk.get(0));
        //config hints
        PMap hints = new PMap();
        hints.putObject("profile", profile);
        KMM matching = new KMM(hopper,hints,chGraph,config);
        long t1 = System.currentTimeMillis();
        MatchResult matchResult = matching.match(observations);
        long t2 = System.currentTimeMillis();
        long matchTime = t2 - t1;
        //format response
        response.type("application/json");
        Map<String, Object> matchStatistics = new HashMap<>();
        Translation tr = new TranslationMap().getWithFallBack(Helper.getLocale("en"));
        RamerDouglasPeucker simplifyAlgo = new RamerDouglasPeucker().setMaxDistance(1.0);
        PathMerger pathMerger = new PathMerger(matchResult.getGraph(), matchResult.getWeighting()).
                setEnableInstructions(false).
                setPathDetailsBuilders(hopper.getPathDetailsBuilderFactory(), new ArrayList<>()).
                setRamerDouglasPeucker(simplifyAlgo).
                setSimplifyResponse(true);
        ResponsePath responsePath = pathMerger.doWork(PointList.EMPTY, Collections.singletonList(matchResult.getMergedPath()),
                hopper.getEncodingManager(), tr);
        List<Double> latitudes = new ArrayList<>();
        List<Double> longitudes = new ArrayList<>();
        for(int i=0;i<responsePath.getPoints().size();i++){
            latitudes.add(responsePath.getPoints().getLat(i));
            longitudes.add(responsePath.getPoints().getLon(i));
        }
        matchStatistics.put("latitudes", latitudes);
        matchStatistics.put("longitudes", longitudes);
        matchStatistics.put("duration", matchTime);
        matchStatistics.put("keypointNum",matchResult.keypointNums);
        System.out.println("Matching time: "+(matchTime));

        Gson gson = new Gson();
        return gson.toJson(matchStatistics);
    }

    public static void main(String[] args) {
        //load Graphhopper
        GraphHopperConfig config = new GraphHopperConfig();
        config.putObject("import.osm.ignored_highways","");
        config.putObject("datareader.file", "Maps/shanghai-latest.osm.pbf"); //change to your own map file
        config.putObject("graph.flag_encoders", "car");
        config.putObject("graph.location", "MapCache");
        config.putObject("datareader.dataaccess", "RAM");
        List<Profile> Profiles = new ArrayList<>();
        Profiles.add(new Profile("car").setCustomModel(new CustomModel().setDistanceInfluence(0.0)).setVehicle("car").setTurnCosts(false));

        config.setProfiles(Profiles);
        hopper = new GraphHopper().init(config);
        hopper.importOrLoad();
        PMap hints = new PMap();
        hints.putObject("profile", "car");
        Weighting weighting = hopper.createWeighting(hopper.getProfile("car"), hints);
        CHConfig chConfig = CHConfig.nodeBased("car", weighting);
        chGraph = prepareCH(hopper.getBaseGraph(), chConfig);

        // start web service
        Spark.port(8999);
        Spark.post("/match", KMMApplication::matchingRoute);
    }

    public static RoutingCHGraph prepareCH(BaseGraph graph, CHConfig chConfig) {
        graph.freeze();
        PrepareContractionHierarchies pch = PrepareContractionHierarchies.fromGraph(graph, chConfig);
        PrepareContractionHierarchies.Result res = pch.doWork();
        return RoutingCHGraphImpl.fromGraph(graph, res.getCHStorage(), res.getCHConfig());
    }
}