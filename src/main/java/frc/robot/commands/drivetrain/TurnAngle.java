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
import frc.robot.subsystems.RamseteDriveSubsystem;

//This command has been overwritten by a command in a different branch... Delete this later
public class TurnAngle extends PIDCommand {

  private RamseteDriveSubsystem m_driveTrain;

  private final static double kP_TURN = 0.1;
  private final static double kI_TURN = 0;
  private final static double kD_TURN = 0;

  private final static AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 200);

  public TurnAngle(RamseteDriveSubsystem driveTrain, double targetAngleDegrees) {
    super(
      new PIDController(kP_TURN, kI_TURN,kD_TURN),
      driveTrain::getHeading,
      targetAngleDegrees,
      output -> driveTrain.tankDrivePower(-output, output),
      driveTrain
    );
    this.m_driveTrain = driveTrain;

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(5.0f);

    addRequirements(m_driveTrain);

  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    navX.zeroYaw();
  }

  // // Called repeatedly when this Command is scheduled to run
   @Override
   public void execute() {
     super.execute();
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
      this.m_driveTrain.arcadeDrive(0, 0, true);
  }


} 