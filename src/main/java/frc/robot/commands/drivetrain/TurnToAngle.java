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
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RamseteDriveSubsystem;

public class TurnToAngle extends PIDCommand {

  private RamseteDriveSubsystem m_driveTrain;

  private final static double kP_TURN = 0.007;
  private final static double kI_TURN = 0;
  private final static double kD_TURN = 0;

  private final static AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 200);

  private Limelight limelight;

  public TurnToAngle(RamseteDriveSubsystem driveTrain, Limelight limelight) {
    super(
      new PIDController(kP_TURN, kI_TURN,kD_TURN),
      navX::getYaw,
      limelight::getHorizontalOffset,
      output -> driveTrain.tankDrivePower(-output, output),
      driveTrain
    );
    this.m_driveTrain = driveTrain;
    this.limelight = limelight;

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(2.0f);

    addRequirements(m_driveTrain);
    addRequirements(limelight);

  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    navX.zeroYaw();
  }

  // // Called repeatedly when this Command is scheduled to run
  // @Override
  // public void execute() {
  //   m_driveTrain.turnToAngle();
  // }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }


  //called once command is interrupted or ended
  @Override
  public void end(boolean interrupted) {
      super.end(interrupted);
      this.m_driveTrain.tankDrivePower(0,0);
  }
}