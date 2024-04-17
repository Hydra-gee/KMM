package entities;

import java.util.List;

public class rlInfo {
    public List<Double> state;
    public double reward;
    public boolean done;
    public List<Float> lats;
    public List<Float> lons;

    public rlInfo(List<Double> state, double reward, boolean done, List<Float> lats, List<Float> lons){
        this.state = state;
        this.reward = reward;
        this.done = done;
        this.lats = lats;
        this.lons = lons;
    }
}
