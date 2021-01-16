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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.VisionUtils;


public class TargetPositioning extends CommandBase {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("limelight");
  private static double KpTurn = 0.1;
  private static double KpDistance = 0.08;
  private static double min_command = 0.05;
  // the range you want.
  private double targetDistance;
  //allowed margin of error
  private double marginOfErrorDist = 5.0;
  private double marginOfErrorTurn = 2.0;
  
  
  // private final RamseteDriveSubsystem m_driveTrain;
  private final DriveTrain m_driveTrain;
  /**
   * Creates a new TargetRange.
   */
  public TargetPositioning(DriveTrain driveTrain, double targetDistance) {
    addRequirements(driveTrain);
    this.m_driveTrain = driveTrain;
    this.targetDistance = targetDistance;
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
    System.out.println("*********************************************");
    
    double ty = (double) table.getEntry("ty").getNumber(0);
    double currentDistance = VisionUtils.getDistanceToTarget(ty);
    double distanceError =   currentDistance - this.targetDistance;
    double tv = (double) table.getEntry("tv").getNumber(0);

    System.out.println("ty"+ty);
    SmartDashboard.putNumber("ty", ty);
    System.out.println("currentDistance"+currentDistance);
    SmartDashboard.putNumber("currentDistance",currentDistance);
    System.out.println("distance error"+distanceError);
    SmartDashboard.putNumber("distance error",distanceError);
    System.out.println(tv);
    SmartDashboard.putNumber("tv", tv);
    
    

    /*if(distanceError<marginOfErrorDist){
      distanceError = 0;
    }*/
    if(tv < 1){
      distanceError = 0;
    }

    double drivingAdjust = KpDistance * distanceError;


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
    m_driveTrain.arcadeDrive(0,steeringAdjust,Constants.kDriveDeadband);
    System.out.println("driving assist"+drivingAdjust);
    SmartDashboard.putNumber("driving assist", drivingAdjust);
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
    double ty = (double) table.getEntry("ty").getNumber(0);
    double tx = (double) table.getEntry("tx").getNumber(0);
    boolean distanceOK =  Math.abs(VisionUtils.getDistanceToTarget(ty)) <= marginOfErrorDist;
    boolean yawOK = Math.abs(tx) <= marginOfErrorTurn;
    return(distanceOK && yawOK);
  }
}