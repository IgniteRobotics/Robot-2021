// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.RamseteDriveSubsystem;

public class TurnAngleEncodersOnly extends CommandBase {
  private RamseteDriveSubsystem driveSubsystem;
  private double leftEncoderTarget;
  private double rightEncoderTarget;

  private double kP = 0.003;

  public TurnAngleEncodersOnly(RamseteDriveSubsystem driveSubsystem, double angleChange) {
    this.driveSubsystem = driveSubsystem;

    double encoderChange = (((angleChange % 360) * Constants.kTrackwidthMeters) / (360 * Constants.WHEEL_DIAMETER)) * Constants.ENCODER_TICKS_PER_REVOLUTION_FALCON;
    leftEncoderTarget = driveSubsystem.getLeftEncoderPosition() + encoderChange;
    rightEncoderTarget = driveSubsystem.getRightEncoderPosition() - encoderChange;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftError = leftEncoderTarget - driveSubsystem.getLeftEncoderPosition();
    double rightError = rightEncoderTarget - driveSubsystem.getRightEncoderPosition();

    driveSubsystem.tankDrivePower(leftError * kP, rightError * kP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
