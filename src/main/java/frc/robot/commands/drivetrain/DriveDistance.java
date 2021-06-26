// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.RamseteDriveSubsystem;


public class DriveDistance extends CommandBase {
  private RamseteDriveSubsystem m_drivetrain;
  private double setpointMeters;
  private double ticksPerMeter =  (2048.0 * 7.8) / Constants.WHEEL_CIRCUMFERENCE_METERS;
  private double targetPositionRotations;
  /** Creates a new DriveDistance. */
  public DriveDistance(double setpointMeters, RamseteDriveSubsystem m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    this.m_drivetrain = m_drivetrain;
    this.setpointMeters = setpointMeters;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
        // double metersPerTick = Constants.WHEEL_CIRCUMFERENCE_METERS / (2048.0 * 7.8);
   
        // double ratio = 7.8; //Gear ratio of robot. Is this correct? input / output?
       //  rightMaster.setInverted(true);
     targetPositionRotations = setpointMeters * ticksPerMeter;
   
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveDistance(targetPositionRotations);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
}
}
