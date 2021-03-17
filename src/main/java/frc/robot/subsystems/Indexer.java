// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  //TODO confirm motor controllers
  private WPI_TalonSRX indexerMaster = new WPI_TalonSRX(Constants.kIndexerMotorPortLeft); //TODO CHANGE THESE TO GOOD CAN IDS
  private WPI_TalonSRX  indexerFollower = new WPI_TalonSRX(Constants.kIndexerMotorPortRight);



  public Indexer() {
    indexerFollower.setInverted(true);
  }
  public void runIndexer(double speed) { //more like percent output

    indexerMaster.set(ControlMode.PercentOutput, speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
