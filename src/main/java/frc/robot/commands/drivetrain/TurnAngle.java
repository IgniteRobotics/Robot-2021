/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.RamseteDriveSubsystem;

//This command has been overwritten by a command in a different branch... Delete this later
public class TurnAngle extends PIDCommand {

  private RamseteDriveSubsystem m_driveTrain;
  private double targetAngleDegrees;

  private final static double kP_TURN = 0.1;
  private final static double kI_TURN = 0;
  private final static double kD_TURN = 0;

  public TurnAngle(RamseteDriveSubsystem driveTrain, double targetAngleDegrees) {
    super(
      new PIDController(kP_TURN, kI_TURN,kD_TURN),
      driveTrain::getHeading,
      driveTrain.getHeading() + targetAngleDegrees,
      output -> driveTrain.arcadeDrive(0, output * 0.1, false),
      driveTrain
    );
    this.m_driveTrain = driveTrain;

    getController().setP(0.05);
    getController().setTolerance(2.0f);

    addRequirements(m_driveTrain);
    this.targetAngleDegrees = targetAngleDegrees;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    super.initialize();
    getController().setSetpoint(m_driveTrain.getHeading() + targetAngleDegrees);
  }

  // // Called repeatedly when this Command is scheduled to run
   @Override
   public void execute() {
     super.execute();
     SmartDashboard.putNumber("TurnAngleError", getController().getPositionError());
   //System.out.println(m_driveTrain.getHeading() + "DAASSS");
   }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }


  //called once command is interrupted or ended
  @Override
  public void end(boolean interrupted) {
     // super.end(interrupted);
      
  }


} 