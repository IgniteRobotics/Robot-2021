/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistance extends CommandBase {
  /**
   * Creates a new DriveToDistance.
   */
  private DriveTrain driveTrain;

  private int setpointInches;

  public DriveToDistance(DriveTrain driveTrain, int setpointInches) {
    this.driveTrain = driveTrain;
    this.setpointInches = setpointInches;

    addRequirements(this.driveTrain);
  }



  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    driveTrain.zeroSensors();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    driveTrain.setMotionMagicPosition(setpointInches);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return driveTrain.isMotionMagicDone();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

}
