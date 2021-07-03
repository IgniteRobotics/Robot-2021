// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;


public class ClimbMotionMagicUp extends CommandBase {
  private Climber climb;
  private int targetPositionTicks = 210000;
  /** Creates a new ClimbMotionMagicUp. */
  public ClimbMotionMagicUp(Climber climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setMotionMagicPosition(targetPositionTicks);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
