package frc.robot.util.shooter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

import frc.robot.util.shooter.ShooterParameter;

public class InterpolationCalculator implements ShooterCalculator {
    private static SortedMap<Double, ShooterParameter> distanceMap = new TreeMap<>();

    static {
        distanceMap.put(0.0, new ShooterParameter(0, 2000, 50));
    }

    // https://theeducationlife.com/interpolation-formula/

    @Override
    public ShooterParameter calculateParameter(double distance) {
        // does treemap iterate in sorted order????
        // what do we do if distance is outside bounds
        List<ShooterParameter> sortedParameters = new ArrayList<>(distanceMap.values());

        // if we're trying to calculate the shooter parameters for a distance that is less/greater than the min/max distances, just return the min/max parameter.
        if(distance < sortedParameters.get(0).distance) {
            return sortedParameters.get(0);
        } else if(distance > sortedParameters.get(sortedParameters.size() - 1).distance) {
            return sortedParameters.get(sortedParameters.size() - 1);
        }

        ShooterParameter lower = null;
        ShooterParameter upper = null;

        for(int i = 0; i < sortedParameters.size(); i++) {
            if(sortedParameters.get(i).distance > distance) {
                lower = sortedParameters.get(i - 1);
                upper = sortedParameters.get(i);
                break;
            }
        }

        // lower and upper should never be null unless I made a mistake
        ShooterParameter interpolated = lower.interpolate(upper, distance - lower.distance);

        return interpolated;
    }
}
