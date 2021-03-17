// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootBall extends CommandBase {
  private Shooter shooter;

  private ShuffleboardTab tab;
  private NetworkTableEntry targetShooterVelocity;

  private static final int RANGE = 50;

  public ShootBall(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tab = Shuffleboard.getTab("Shooter");
    targetShooterVelocity = tab.add("Target Shooter Velocity", 0).withProperties(Map.of("min", 0)).getEntry();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetVelocity = targetShooterVelocity.getDouble(0);
    // get velocity from the Shuffleboard
    setShooterVelocity(targetVelocity);

    if(targetVelocity - RANGE < shooter.getShooterRPM() && targetVelocity + RANGE > shooter.getShooterRPM()) {
      shooter.runKickup();
    } else {
      shooter.stopKickup();
    }
  }

  private void setShooterVelocity(double velocity) {
    if(velocity >= 0) {
      shooter.setVelocity(velocity);
    }
  }

  private void stop() {
    shooter.setVelocity(0);
    shooter.stopKickup();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop();
  }

  @Override
  public void cancel() {
    // just in case, to ensure motor stops
    stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
