/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  
  private DriveTrain driveTrain;
  
  private double setpoint;
  /**
   * Creates a new TurnToAngle.
   */
  public TurnToAngle(DriveTrain driveTrain, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.setpoint = setpoint;
    
    requires(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //todo make the commands refrenced
    //driveTrain.zeroAngle();
    //driveTrain.enableTurnController(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //driveTrain.TurnToAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //driveTrain.stopTurnController();
    //driveTrain.stop();
    //driveTrain.zeroAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return driveTrain.isTurnCompleted();
  }
}
