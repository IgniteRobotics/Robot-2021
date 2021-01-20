/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {

  private DriveTrain m_driveTrain;

  private double setpoint;

  public TurnToAngle(DriveTrain driveTrain, double setpoint) {
    m_driveTrain = driveTrain;
    this.setpoint = setpoint;

    addRequirements(m_driveTrain);

  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    m_driveTrain.zeroAngle();
    m_driveTrain.enableTurnController(setpoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_driveTrain.turnToAngle();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return m_driveTrain.isTurnCompleted();
  }

  //called once command is interrupted or ended
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stopTurnController();
    m_driveTrain.stop();
    m_driveTrain.zeroAngle();
  }

}