// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RamseteDriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivePathAndShoot extends CommandBase {
  /** Creates a new DrivePathAndShoot. */
  //Runs during autonomous period of july 2021 competition. Select trajectory to run on shuffleboard
private RamseteDriveSubsystem m_driveTrain;
private Intake m_intake;

 
public DrivePathAndShoot(RamseteDriveSubsystem drivetrain, Intake intake) {
  this.m_driveTrain = drivetrain;
  this.m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Load path
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
