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
//Shoots the ball at a specific RPM and angle. RPM and angle are set from shuffleboard
public class ShootBallTest extends CommandBase {
    private Shooter shooter;
    private Indexer indexer;

    private ShuffleboardTab tab;
    private NetworkTableEntry targetShooterVelocityEntry;
    private NetworkTableEntry intakeEffortEntry;
    private NetworkTableEntry kickupEffortEntry;
    private NetworkTableEntry distanceSetPointEntry;

    private Limelight limelight;

    private StateMachine state;

    private static final int RANGE = 150;
    private static final double ANGLE_RANGE = 5;

    private double targetVelocity = Constants.HOOD_DEFAULT_RPM;
    private double intakeEffort = 0.4;
    private double kickupEffort = 0.3;
 //   private double distanceSetpoint = Constants.HOOD_SET_POINT_DISTANCE;

    public ShootBallTest(Shooter shooter, Indexer indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter);
  
        //This might conflict with ShootinterpolatedBall... TODO check
        //Consider moving this to robotcontainer 
        tab = Shuffleboard.getTab("Shooter");
        targetShooterVelocityEntry = tab.add("Target Shooter Velocity", Constants.SHOOTER_DEFAULT_RPM).withProperties(Map.of("min", 0)).getEntry();
        distanceSetPointEntry = tab.add("Shooter Distance Setpoint", Constants.HOOD_SET_POINT_DISTANCE).withProperties(Map.of("min", 0)).getEntry();
        intakeEffortEntry = tab.add("Intake Effort Percentage", 0.6).withProperties(Map.of("min", -1, "max", 1)).getEntry();
        kickupEffortEntry = tab.add("Kickup Wheel Effort Percentage", 0.5).withProperties(Map.of("min", 0, "max", 1)).getEntry();

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
        targetVelocity = targetShooterVelocityEntry.getDouble(Constants.HOOD_DEFAULT_RPM);
        intakeEffort = intakeEffortEntry.getDouble(0.4);
        kickupEffort = kickupEffortEntry.getDouble(0.3);
      //unused for now  distanceSetpoint = distanceSetPointEntry.getDouble(Constants.HOOD_SET_POINT_DISTANCE);
        //double currentDistance = state.getShooterDistance(); State machine is not currently being used
        targetVelocity = targetShooterVelocityEntry.getDouble(0.0);
        targetShooterVelocityEntry.setDouble(targetVelocity);

        //error: 75 in -> 1.905m
        //
        // if (currentDistance > distanceSetpoint){
        //   shooter.extendHood();
        // } else {
        //   shooter.retractHood();
        // }


        // get velocity from the Shuffleboard
     //   setShooterVelocity(targetVelocity);
        setShooterRPM((int) targetVelocity); // use rpm from interpolation

        //TODO look at this. Find specific angle to shoot from
        shooter.changeHoodAngle(60);

        double shooterRPM = shooter.getShooterRPM();

        // if((targetVelocity - RANGE <= shooterRPM && targetVelocity + RANGE >= shooterRPM) &&
        //     (shooterAngle - ANGLE_RANGE < computedAngle && shooterAngle + ANGLE_RANGE > computedAngle)) {
        if ((targetVelocity - RANGE <= shooterRPM && targetVelocity + RANGE >= shooterRPM)
            //  && shooter.isHoodReady()
        ) {
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