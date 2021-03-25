// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class TestRetractHood extends CommandBase {

  private Shooter m_shooter;

  /** Creates a new TestRetractHood. */
  public TestRetractHood(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.retractHood();;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double range = 5;
    return m_shooter.getHoodTicks() - range <= 0 
    && m_shooter.getHoodTicks() + range >= 0;
  }
}
