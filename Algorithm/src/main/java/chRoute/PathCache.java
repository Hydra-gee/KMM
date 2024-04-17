package chRoute;

import com.graphhopper.routing.Path;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PathCache {
    public List<Path> pathList;
    public Map<Integer,List<Integer>> pathIndexMap;
    public Map<Integer,Map<Integer,Integer>> posIndexMap;
    public int cacheType;
    public PathCache() {
        pathList = new ArrayList<>();
        pathIndexMap = new HashMap<>();
        posIndexMap = new HashMap<>();
    }
}
