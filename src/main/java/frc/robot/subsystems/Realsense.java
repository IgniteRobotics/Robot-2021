// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.PathData;

public class Realsense extends SubsystemBase {

  public boolean hasTarget = false;

  double distance = -1;
  double offset = -1;

  NetworkTableInstance networkTableInst;
  NetworkTable table;
  ShuffleboardTab shuffleboardTab;
  

  /** Creates a new Realsense. */
  public Realsense() {

    distance = table.getEntry("dist").getDouble(-1.0);
    offset = table.getEntry("offset").getDouble(-1.0);

    networkTableInst = NetworkTableInstance.getDefault();
    table = networkTableInst.getTable("galacticsearch");
    shuffleboardTab = Shuffleboard.getTab("Galactic Search");
    SmartDashboard.putString("Robot-Path-Determination-Inator", "Selecting Path...");

  }

  public Trajectory determinePath() {

    PathData[] paths = {Constants.pathARed, Constants.pathBRed, Constants.pathABlue, Constants.pathBBlue};

    if(distance == -1.0) {
      DriverStation.reportError("There was an error getting the ball distance!", false);
      return null;
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
        //SmartDashboard.putString("Error of Chosen Path", chosenPath.getTrajectory());
        }
      }
    
    //display chosen path
    SmartDashboard.putString("Robot-Path-Determination-Inator", chosenPath.getTrajectory());

    Trajectory chosenTrajectory = RobotContainer.loadTrajectory(chosenPath.getTrajectory());

    hasTarget = true;
    return chosenTrajectory; 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distance = table.getEntry("dist").getDouble(-1.0);
    offset = table.getEntry("offset").getDouble(-1.0);
  }
}
