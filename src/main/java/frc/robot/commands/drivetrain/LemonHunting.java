// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RamseteDriveSubsystem;

public class LemonHunting extends CommandBase {
  RamseteDriveSubsystem m_drivetrain;
  Intake m_intake;


  double KpAim = -0.1;
  double KpDistance = -0.1;
  double minAimCommand = 0.05;
  double drivePower = 0.3;

  NetworkTable realsenseData;
  String tableName = "vision";
  double tx = 0;
  double ty = 0;
  double dist = 0;

  //shuffleboardstuff
  private ShuffleboardTab tab;
  private NetworkTableEntry kpAimEntry;
  private NetworkTableEntry kpDistanceEntry;
  private NetworkTableEntry drivePowerEntry;

  /** Creates a new LemonHunting. */
  public LemonHunting(RamseteDriveSubsystem drivetrain, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, intake);
    this.m_drivetrain = drivetrain;
    this.m_intake = intake;
    realsenseData = NetworkTableInstance.getDefault().getTable(tableName);

    //setup shuffleboard
    tab = Shuffleboard.getTab("LemonHunter");
    kpAimEntry = tab.add("kP Aim", KpAim).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    kpDistanceEntry = tab.add("kPDistance", KpDistance).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    drivePowerEntry = tab.add("Drive Power Percentage", drivePower).withProperties(Map.of("min", 0, "max", 1)).getEntry();

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.tx = 0;
    this.ty = 0;
    this.dist = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tx = realsenseData.getEntry("yaw").getDouble(0);
    this.ty = realsenseData.getEntry("pitch").getDouble(0);
    this.dist = realsenseData.getEntry("dist").getDouble(0);
    double headingError = -tx;
    double steeringAdjust = 0;

    this.m_intake.spin(0.4);

    double leftPower;
    double rightPower;


    //if there's a target, go toward it.
    if (this.dist > 0){
      if (tx > 1.0)
      {
              steeringAdjust = KpAim*headingError - minAimCommand;
      }
      else if (tx < 1.0)
      {
              steeringAdjust = KpAim*headingError + minAimCommand;
      }

      leftPower = steeringAdjust + drivePower;
      rightPower = -(steeringAdjust + drivePower);

    } else {  //no target.  seek.
      steeringAdjust = 0.3;
      //NOTE getYaw is negated in the drive train.
      // +yaw is LEFT turn
      // -yaw is RIGHT turn
      if (m_drivetrain.getYaw() < 0){ // bot is turned to the right of center field so reverse
        steeringAdjust *= -1;
      }
      leftPower = steeringAdjust;
      rightPower = -steeringAdjust;
    }

    
    m_drivetrain.tankDrivePower(leftPower, rightPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_drivetrain.stop();
    this.m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
