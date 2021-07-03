// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.util.StateMachine;
import frc.robot.util.shooter.InterpolationCalculator;
import frc.robot.util.shooter.ShooterParameter;

import java.util.Map;
//Shoots the ball at a specific RPM and angle. RPM and angle set during command instantiation
public class ShootBallSpecific extends CommandBase {
    private Shooter shooter;
    private Indexer indexer;

    private Limelight limelight;

    private StateMachine state;

    private static final int RANGE = 150;
    private static final double HOOD_TOL = 5;

    private double targetVelocity;
    private double intakeEffort = 0.4;
    private double kickupEffort = 0.3;
    private  int targetHoodTicks;
 //   private double distanceSetpoint = Constants.HOOD_SET_POINT_DISTANCE;

    public ShootBallSpecific(Shooter shooter, Indexer indexer, int targetRPM, int targetHoodTicks) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.targetVelocity = targetRPM;
        this.targetHoodTicks = targetHoodTicks;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // tab = Shuffleboard.getTab("Shooter");
        // targetShooterVelocityEntry = tab.add("Target Shooter Velocity", 0).withProperties(Map.of("min", 0)).getEntry();
        // intakeEffortEntry = tab.add("Intake Effort Percentage", 0.4).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        // kickupEffortEntry = tab.add("Kickup Wheel Effort Percentage", 0.3).withProperties(Map.of("min", 0, "max", 1)).getEntry();
        state = StateMachine.getIntance();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        setShooterRPM((int) targetVelocity); // use rpm from interpolation

        //TODO look at this. Find specific angle to shoot from
        shooter.changeHoodTicks(targetHoodTicks);

        double shooterRPM = shooter.getShooterRPM();

        // if((targetVelocity - RANGE <= shooterRPM && targetVelocity + RANGE >= shooterRPM) &&
        //     (shooterAngle - ANGLE_RANGE < computedAngle && shooterAngle + ANGLE_RANGE > computedAngle)) {
        if (targetVelocity - RANGE <= shooterRPM && targetVelocity + RANGE >= shooterRPM && shooter.isHoodReady()) {
            shooter.runKickup(kickupEffort);
            indexer.runIndexer(intakeEffort);
        } else {
            shooter.stopKickup();
            indexer.runIndexer(0);
        }
    }

    private void setShooterVelocity(double velocity) {
        if (velocity >= 0) {
            shooter.setVelocity(velocity);
        }
    }

    private void setShooterRPM(int rpm) {
        if (rpm >= 0) {
            shooter.setRPM(rpm);
        } else {
            shooter.setRPM(0);
        }
    }


    private void stop() {
        shooter.setpower(0.0);
        shooter.stopKickup();
        indexer.stop();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
