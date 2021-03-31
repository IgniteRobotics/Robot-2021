// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.PathData;


//remember to place the robot directly in front of the closest ball 
public class DrivegalacticRun extends CommandBase {
  private NetworkTableInstance networkTableInst = NetworkTableInstance.getDefault();
  private NetworkTable table = networkTableInst.getTable("galacticsearch");
  private ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Galactic Search");
  
  




  private PathData[] paths = {Constants.pathARed, Constants.pathBRed, Constants.pathABlue, Constants.pathBBlue};

  double distance = -1;
  double offset = -1;
  
  /** Creates a new DrivegalacticRun. */
  public DrivegalacticRun(double distance, double offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    // initialize shuffleboard parameters
    this.distance = distance;
    this.offset = offset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(distance == -1 || offset == -1) {
      DriverStation.reportError("There was an error getting the ball distance!", false);
      return;
    }

    //TODO replace this null or maybe refactor?
    PathData sensedData = new PathData(distance, offset, null);
    PathData chosenPath = paths[0];

    double minError = paths[0].getPathError(sensedData);
    for(int i = 1; i < paths.length; i++) {
      double error = paths[i].getPathError(sensedData);
      if(error < minError) {
        minError = error;
        chosenPath = paths[i];
      }
    }

    Trajectory chosenTrajectory = RobotContainer.loadTrajectory(chosenPath.getTrajectory());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // follow the path and pick up the balls
    //check if ball has been picked up
    
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
