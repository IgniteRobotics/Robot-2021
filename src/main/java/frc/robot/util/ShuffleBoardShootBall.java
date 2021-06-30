package frc.robot.util;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants;
import java.util.Map;

public class ShuffleBoardShootBall {
    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    private NetworkTableEntry targetShooterVelocityEntry;
    private NetworkTableEntry distanceSetPointEntry;
    private NetworkTableEntry intakeEffortEntry;
    private NetworkTableEntry kickupEffortEntry;

    private static ShuffleBoardShootBall instance;

    private ShuffleBoardShootBall() {
        targetShooterVelocityEntry = tab.add("Target Shooter Velocity", Constants.SHOOTER_DEFAULT_RPM).withProperties(Map.of("min", 0)).getEntry();
        distanceSetPointEntry = tab.add("Shooter Distance Setpoint", Constants.HOOD_SET_POINT_DISTANCE).withProperties(Map.of("min", 0)).getEntry();
        intakeEffortEntry = tab.add("Intake Effort Percentage", 0.6).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        kickupEffortEntry = tab.add("Kickup Wheel Effort Percentage", 0.5).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    }

    public double getTargetVelocity() {
        return targetShooterVelocityEntry.getDouble(0);
    }

    public double getIntakeEffort() {
        return intakeEffortEntry.getDouble(0);
    }

    public double getKickupEffort() {
        return kickupEffortEntry.getDouble(0);
    }

    public double getDistanceSetPoint() {
        return distanceSetPointEntry.getDouble(0);
    }

    public static synchronized ShuffleBoardShootBall getInstance() {
        if(instance == null) {
            instance = new ShuffleBoardShootBall();
        }

        return instance;
    }
}