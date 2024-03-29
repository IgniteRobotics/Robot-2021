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
    private static double goalOffset = -.737;
    static {
/**         distanceMap.put(6.0, new ShooterParameter(6, 5000, 1633));
        distanceMap.put(8.0, new ShooterParameter(8, 6000, 1587));
        distanceMap.put(3.0, new ShooterParameter(3, 4000, 1091));
        distanceMap.put(1.0, new ShooterParameter(1, 3500, 484));

        */
        // addShooterParameter(new ShooterParameter(5.388, 5000, 1598));
        // addShooterParameter(new ShooterParameter(3.34, 3800, 734));
        // addShooterParameter(new ShooterParameter(3.86, 5000, 1598));

        addShooterParameter(new ShooterParameter(1.33 + goalOffset, 3500, 140));
        addShooterParameter(new ShooterParameter(2.1 + goalOffset, 3600, 195));
        addShooterParameter(new ShooterParameter(4.13 + goalOffset, 4800, 1600));
        addShooterParameter(new ShooterParameter(4.75 + goalOffset, 4650, 1600));
        addShooterParameter(new ShooterParameter(5.55 + goalOffset, 4850, 1600));
        addShooterParameter(new ShooterParameter(6.32 + goalOffset, 5000, 1600));
        addShooterParameter(new ShooterParameter(6.89 + goalOffset, 5200, 1600));
        addShooterParameter(new ShooterParameter(7.60 + goalOffset, 5900, 1600));
        addShooterParameter(new ShooterParameter(8.60 + goalOffset, 5500, 1600));
        

    }

    // https://theeducationlife.com/interpolation-formula/

    private static void addShooterParameter(ShooterParameter parameter) {
        distanceMap.put(parameter.distance, parameter);
    }

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
