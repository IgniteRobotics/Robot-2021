/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RamseteDriveSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.VisionUtils;


public class TargetPositioning extends CommandBase {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("limelight");
  private static double KpTurn = 0.05;
  private static double min_command = 0.05;
  // the range you want.
  //allowed margin of errorit 
  private double marginOfErrorTurn = 2.0;
  
  
  private final RamseteDriveSubsystem m_driveTrain;
  /**
   * Creates a new TargetRange.
   */
  public TargetPositioning(RamseteDriveSubsystem driveTrain) {
    addRequirements(driveTrain);
    this.m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called wen the command is initially scheduled.
  @Override
  public void initialize() {
    table.getEntry("camMode").setNumber(0);
    table.getEntry("ledMode").setNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double ty = (double) table.getEntry("ty").getNumber(0);
    double tv = (double) table.getEntry("tv").getNumber(0);

    SmartDashboard.putNumber("ty", ty);
    SmartDashboard.putNumber("tv", tv);
    
    




    double tx = (double) table.getEntry("tx").getNumber(0);
    double headingError = -tx;
    double steeringAdjust = 0.0;
    if (tx > 1.0)
    {
             steeringAdjust =  KpTurn*headingError - min_command;
    }
    else if (tx < 1.0)
    {
            steeringAdjust = KpTurn*headingError + min_command;
    }


    //m_driveTrain.arcadeDrive(-drivingAdjust,steeringAdjust,Constants.kDriveDeadband);
    m_driveTrain.arcadeDrive(0,steeringAdjust,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    table.getEntry("camMode").setNumber(1);
    table.getEntry("ledMode").setNumber(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double tx = (double) table.getEntry("tx").getNumber(0);
    boolean yawOK = Math.abs(tx) <= marginOfErrorTurn;
    return yawOK;
  }
}