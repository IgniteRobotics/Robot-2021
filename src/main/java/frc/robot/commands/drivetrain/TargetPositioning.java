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

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.VisionUtils;


public class TargetPositioning extends CommandBase {
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static NetworkTable table = inst.getTable("limelight");
  private static double KpTurn = 0.02;
  private static double min_command = 0.05;
  // the range you want.
  //allowed margin of errorit 
  private double marginOfErrorTurn = 2.0;

  private ShuffleboardTab tab;
  private NetworkTableEntry kpTurnEntry;

  boolean targetFound = false;
  

  
  private final RamseteDriveSubsystem m_driveTrain;
  /**
   * Creates a new TargetRange.
   */
  public TargetPositioning(RamseteDriveSubsystem driveTrain) {
    addRequirements(driveTrain);
    this.m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.

    tab = Shuffleboard.getTab("Limelight");
    kpTurnEntry = tab.add("kPTurn Limelight", KpTurn).withProperties(Map.of("min", 0)).getEntry();

  }

  // Called wen the command is initially scheduled.
  @Override
  public void initialize() {
    targetFound = false;
    KpTurn = kpTurnEntry.getDouble(KpTurn);
    table.getEntry("camMode").setNumber(0);
    table.getEntry("ledMode").setNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    double tv = (double) table.getEntry("tv").getNumber(0);
    SmartDashboard.putNumber("Limelight tv", tv);

    if (tv > 0) {
      
      targetFound = true;

      double tx = (double) table.getEntry("tx").getNumber(0);
      double ty = (double) table.getEntry("ty").getNumber(0);

      SmartDashboard.putNumber("Limelight tx", tx);
      SmartDashboard.putNumber("Limelight ty", ty);
      
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
      //flip it since the shooter is on the back.
      m_driveTrain.arcadeDrive(0,-steeringAdjust,true);
    } else {
      targetFound = false;
    }
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
    return yawOK && targetFound;
  }
}