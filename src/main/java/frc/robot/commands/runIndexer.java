// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class runIndexer extends CommandBase {
  /** Creates a new runIndexer. */
  private double percent_output;
  private Indexer indexer;
  public runIndexer(double percent_output, Indexer m_indexer) { 
    // Use addRequirements() here to declare subsystem dependencies.
    this.percent_output = percent_output;
    indexer = m_indexer;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.runIndexer(this.percent_output); 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    indexer.runIndexer(this.percent_output);
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
