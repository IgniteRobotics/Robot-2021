// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

//remember to place the robot directly in front of the closest ball 
public class DrivegalacticRun extends CommandBase {
  private NetworkTableInstance networkTableInst = NetworkTableInstance.getDefault();
  private NetworkTable table = networkTableInst.getTable("galacticsearch");
  
  // dummy values - not actually measured
  private PathData pathARed = new PathData(90, 0);
  private PathData pathBRed = new PathData(90, -200);

  /**
   * Represents the unique characteristics for each path in order to determine which path to be used.
   * 
   * There are two characteristics that used together can uniquely identify each path - the distance of the closest ball,
   * and the offset of the closest ball (in px). For example, the red spots on path a could be characterized as 90 cm and 0 px,
   * since the robot would be 90 cm away from the ball and the ball would be in the center of the camera. (The robot should be placed in the middle.)
   */
  public static class PathData {
    private double dist;
    private double ballOffset; //pixel offset
    public static double DIST_T = 10; // distance tolerance (cm)
    public static double OFFSET_T = 100; // offset tolerance (px) - we should find this value later

    public PathData(double dist, double ballOffset) {
      this.dist = dist;
      this.ballOffset = ballOffset;
    }

    public double getDist() {
      return this.dist;
    }

    public double getOffset() {
      return this.ballOffset;
    }

    /**
     * Check if a path matches the characteristics of this path.
     */
    public boolean checkPath(PathData d) {
      return ((d.getDist() > (this.dist - DIST_T) && d.getDist() < (this.dist + DIST_T)) && (d.getOffset() > (this.ballOffset - OFFSET_T) && d.getOffset() < (this.ballOffset + OFFSET_T)));
    }
  }
  
  /** Creates a new DrivegalacticRun. */
  public DrivegalacticRun() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // read values from networktables and determine which path to use
    NetworkTableEntry distEntry = table.getEntry("dist"); // this entry contains the distance (cm) of the closest ball relative to the robot.
    NetworkTableEntry xEntry = table.getEntry("offset");
    double distance = distEntry.getDouble(-1);
    double offset = xEntry.getDouble(-1);

    if(distance == -1 || offset == -1) {
      DriverStation.reportError("There was an error getting the ball distance!", false);
      return;
    }

    PathData sensedData = new PathData(distance, offset);

    if(pathARed.checkPath(sensedData)) {
      // run trajectory for path A red points
    } else if(pathBRed.checkPath(sensedData)) {
      // run trajectory for path B red points
    }
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
