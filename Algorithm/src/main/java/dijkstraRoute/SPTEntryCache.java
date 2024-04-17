package dijkstraRoute;

import com.graphhopper.routing.SPTEntry;

import java.util.ArrayList;

public class SPTEntryCache{
    public ArrayList<SPTEntryCache> childs;
    public Long nodeId;
    public SPTEntry node;
    public double weight;
    public SPTEntryCache(SPTEntry node,double weight) {
        this.node = node;
        this.weight = weight;
        this.childs = new ArrayList<>();
    }
}
