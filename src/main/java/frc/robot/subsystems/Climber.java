// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  //Climb won't break if we extend too far. Will it fly off if we let it go up too fast?

  private WPI_TalonFX leftClimb;
  private WPI_TalonFX rightClimb;

  private ShuffleboardTab shuffleTab = Shuffleboard.getTab("Climber");
  private NetworkTableEntry leftClimbTicks = shuffleTab.add("Left Climb (Ticks)", 0).getEntry();
  private NetworkTableEntry rightClimbTicks = shuffleTab.add("Right Climb (Ticks)", 0).getEntry();

  private NetworkTableEntry leftClimbCurrent = shuffleTab.add("Left Climb Supply (Amps)", 0).getEntry();
  private NetworkTableEntry rightClimbCurrent = shuffleTab.add("Right Climb Supply (Amps)", 0).getEntry();

  public Climber() {
      leftClimb = new WPI_TalonFX(MotorConstants.kLeftClimberMotorPort);
      rightClimb = new WPI_TalonFX(MotorConstants.kRightClimberMotorPort);


      //TODO check what direction motor needs to pull to bring in climber
      rightClimb.follow(leftClimb);
      rightClimb.setInverted(true);

      leftClimb.setNeutralMode(NeutralMode.Brake);
      rightClimb.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    publishData();
  }

  private void publishData() {
    leftClimbTicks.setNumber(leftClimb.getSelectedSensorPosition());
    rightClimbTicks.setNumber(rightClimb.getSelectedSensorPosition());

    leftClimbCurrent.setNumber(leftClimb.getSupplyCurrent());
    rightClimbCurrent.setNumber(rightClimb.getSupplyCurrent());
  }

  public void goUp(){
    //TODO make this shuffleboard changeable
    leftClimb.set(ControlMode.PercentOutput, .75);

  }

  public void goDown() {
    leftClimb.set(ControlMode.PercentOutput, -.75);
  }
}
