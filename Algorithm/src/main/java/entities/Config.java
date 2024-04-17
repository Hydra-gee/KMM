package entities;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
@AllArgsConstructor
public class Config {
    private double measurementErrorSigma;
    private boolean useVelocity;
    private double velocityThreshold;
    private double filterThreshold;
    private int maxKeypointNum;
    private int pathCalculateAlgo;
    private int cacheType;
    private int algorithm;
    private double kpDisThreshold;
    private int kpConnumThreshold;
    private int weightType;
}
