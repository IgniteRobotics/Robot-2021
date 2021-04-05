// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.commands.drivetrain.DriveTrajectory;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RamseteDriveSubsystem;
import frc.robot.subsystems.Realsense;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialGalacticSearch extends SequentialCommandGroup {

  private Realsense m_realsense;
  private RamseteDriveSubsystem r_drivetrain;
  private Intake  m_intake;
  private Trajectory pathChosen; 
  private boolean runOnce = true;

  /** Creates a new SequentialGalacticSearch. */
  public SequentialGalacticSearch(RamseteDriveSubsystem r_drivetrain, Realsense m_realsense, Intake  m_intake) {
    // Add your commands in the addCommands() call, e.g.
    //Runs galaticRun command to determine trajectory, and then drives it 

     this.m_realsense = m_realsense;
     this.r_drivetrain = r_drivetrain;
     this. m_intake = m_intake;
     

     
  }
  @Override
  public void initialize(){
  
    //Initally toggle the intake 
   addCommands( new  ToggleIntake(m_intake));  
    //start pathdetermination 
    pathChosen = m_realsense.determinePath();
    
  }
  @Override
  public void execute() {


    if (m_realsense.hasTarget && runOnce)  {

      //When a path is determined, in parallel, run the driveTrajectory when determinePath has a target 
      //and the intake toggle and runIntake
      runOnce = false;
      ParallelCommandGroup parallel = new ParallelCommandGroup( new DriveTrajectory(r_drivetrain, pathChosen), new ToggleIntake(m_intake), new RunIntake(.7, m_intake)); 
      addCommands( parallel );


    }


  }

  @Override
  public boolean isFinished() {
    return false;
  }


}