// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.MotorConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  //TODO confirm motor controllers
  private WPI_VictorSPX indexerMaster = new WPI_VictorSPX(MotorConstants.kIndexerMotorPortLeft); 
  private WPI_VictorSPX indexerFollower = new WPI_VictorSPX(MotorConstants.kIndexerMotorPortRight);



  public Indexer() {
    indexerFollower.setInverted(true);
  }
  public void runIndexer(double speed) { //more like percent output

    indexerMaster.set(ControlMode.PercentOutput, speed);
    indexerFollower.set(ControlMode.PercentOutput, speed);

  }

  public void stop(){
    indexerMaster.set(ControlMode.PercentOutput, 0);
    indexerFollower.set(ControlMode.PercentOutput, 0);
  }

  public double getPower(){
    return indexerMaster.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
