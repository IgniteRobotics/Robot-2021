// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import java.util.Map;

public class ShootBall extends CommandBase {
  private Shooter shooter;
  private Indexer indexer;

  private ShuffleboardTab tab;
  private NetworkTableEntry targetShooterVelocityEntry;
  private NetworkTableEntry intakeEffortEntry;
  private NetworkTableEntry kickupEffortEntry;

  private Limelight limelight;

  private static final int RANGE = 50;
  private static final double ANGLE_RANGE = 5;

  public ShootBall(Shooter shooter, Indexer indexer, Limelight limelight) {
    this.shooter = shooter;
    this.indexer = indexer;
    this.limelight = limelight;
    addRequirements(shooter, indexer);

    tab = Shuffleboard.getTab("Shooter");
    targetShooterVelocityEntry = tab.add("Target Shooter Velocity", 0).withProperties(Map.of("min", 0)).getEntry();
    intakeEffortEntry = tab.add("Intake Effort Percentage", 0.4).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    kickupEffortEntry = tab.add("Kickup Wheel Effort Percentage", 0.3).withProperties(Map.of("min", 0, "max", 1)).getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // tab = Shuffleboard.getTab("Shooter");
    // targetShooterVelocityEntry = tab.add("Target Shooter Velocity", 0).withProperties(Map.of("min", 0)).getEntry();
    // intakeEffortEntry = tab.add("Intake Effort Percentage", 0.4).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    // kickupEffortEntry = tab.add("Kickup Wheel Effort Percentage", 0.3).withProperties(Map.of("min", 0, "max", 1)).getEntry();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetVelocity = targetShooterVelocityEntry.getDouble(1000);
    double intakeEffort = intakeEffortEntry.getDouble(0.4);
    double kickupEffort = kickupEffortEntry.getDouble(0.3);
    double shooterAngle = shooter.getHoodAngle();
    double computedAngle = computeAngleFromDistance();
    // get velocity from the Shuffleboard
    //setShooterVelocity(targetVelocity);
    setShooterRPM((int)targetVelocity);
    // shooter.changeHoodAngle(computedAngle);

    double shooterRPM = shooter.getShooterRPM();

    // if((targetVelocity - RANGE <= shooterRPM && targetVelocity + RANGE >= shooterRPM) &&
    //     (shooterAngle - ANGLE_RANGE < computedAngle && shooterAngle + ANGLE_RANGE > computedAngle)) {
    if((targetVelocity - RANGE <= shooterRPM && targetVelocity + RANGE >= shooterRPM)) {
      shooter.runKickup(kickupEffort);
      indexer.runIndexer(intakeEffort); 
    } else {
      shooter.stopKickup();
      indexer.runIndexer(0);
    }
  }

  private void setShooterVelocity(double velocity) {
    if(velocity >= 0) {
      shooter.setVelocity(velocity);
    }
  }

  private void setShooterRPM(int rpm){
    if (rpm >= 0){
      shooter.setRPM(rpm);
    } else {
      shooter.setRPM(0);
    }
  }

  private double computeAngleFromDistance() {
    // Implementation TODO
    return -1;
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
