/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
//TODO test if this works


/**
 * An example command that uses an example subsystem.
 */
public class ToggleSlowMode extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  DriveTrain m_driveTrain;
  /**
   * Creates a new ExampleCommand.
   *.
   * @param subsystem The subsystem used by this command.
   */
  public ToggleSlowMode(DriveTrain drivetrain) {
     m_driveTrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.toggleSlowMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
@Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.toggleSlowMode(); //Should this go in isFinished?
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //Not sure about this. TODO test if slow mode works
    return true;
  }
}

