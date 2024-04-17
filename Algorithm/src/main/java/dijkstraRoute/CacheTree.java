package dijkstraRoute;

import com.graphhopper.routing.SPTEntry;

import java.util.HashMap;

public class CacheTree {
    public HashMap<Integer,SPTEntryCache> nodeMap;

    public void addNode(SPTEntry node,Integer nodeId,SPTEntry parentNode,Integer parentId,double weight){
        if(parentId == null){
            SPTEntryCache cacheNode = new SPTEntryCache(node,0);
            this.nodeMap.put(nodeId, cacheNode);
        }else{
            if(!this.nodeMap.containsKey(parentId)){
                this.addNode(parentNode,parentId,null,null,0);
            }
            SPTEntryCache pNode = this.nodeMap.get(parentId);
            SPTEntryCache cacheNode = new SPTEntryCache(node,weight);
            pNode.childs.add(cacheNode);
            this.nodeMap.put(nodeId, cacheNode);
        }
    }

    public CacheTree(){
        this.nodeMap = new HashMap<>();
    }
}
