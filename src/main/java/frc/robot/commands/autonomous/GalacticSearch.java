// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.drivetrain.DriveTrajectory;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RamseteDriveSubsystem;
import frc.robot.subsystems.Realsense;

public class GalacticSearch extends CommandBase {
  private RamseteDriveSubsystem m_driveTrain;
  private Realsense m_realsense;
  private Intake m_intake;

  
  
  /** Creates a new GalacticSearchRunBetter. */
  public GalacticSearch(RamseteDriveSubsystem drivetrain, Realsense realsense, Intake intake ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_driveTrain = drivetrain;
    this.m_realsense = realsense;
    this.m_intake = intake;
    addRequirements(drivetrain, intake);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory t = this.m_realsense.determinePath();
    if (t != null){
      ParallelCommandGroup pcg = new ParallelCommandGroup(new RunIntake(1.0, m_intake),
        new DriveTrajectory(this.m_driveTrain, t));
        pcg.schedule();
      end(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
