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

  private WPI_TalonFX climberLeader;
  private WPI_TalonFX climberFollower;

  private ShuffleboardTab shuffleTab = Shuffleboard.getTab("Climber");
  private NetworkTableEntry leftClimbTicks = shuffleTab.add("Left Climb (Ticks)", 0).getEntry();
  private NetworkTableEntry rightClimbTicks = shuffleTab.add("Right Climb (Ticks)", 0).getEntry();

  private NetworkTableEntry leftClimbCurrent = shuffleTab.add("Left Climb Supply (Amps)", 0).getEntry();
  private NetworkTableEntry rightClimbCurrent = shuffleTab.add("Right Climb Supply (Amps)", 0).getEntry();

  public Climber() {
      climberLeader = new WPI_TalonFX(MotorConstants.kLeftClimberMotorPort);
      climberFollower = new WPI_TalonFX(MotorConstants.kRightClimberMotorPort);


      //TODO check what direction motor needs to pull to bring in climber
      climberFollower.follow(climberLeader);
      climberFollower.setInverted(true);

      climberLeader.setNeutralMode(NeutralMode.Brake);
      climberFollower.setNeutralMode(NeutralMode.Brake);
    
       // leftClimb.configMotionCruiseVelocity(CRUISE_VELOCITY, 10);
      //  leftClimb.configMotionAcceleration(MAX_ACCELERATION, 10);

        //Livewindow methods for testing
        addChild("climberLeader- Climber",climberLeader);
        addChild("climberFollower- Climber",climberFollower);

  }

  @Override
  public void periodic() {
    publishData();
  }

  private void publishData() {
    leftClimbTicks.setNumber(climberLeader.getSelectedSensorPosition());
    rightClimbTicks.setNumber(climberFollower.getSelectedSensorPosition());

    leftClimbCurrent.setNumber(climberLeader.getSupplyCurrent());
    rightClimbCurrent.setNumber(climberFollower.getSupplyCurrent());
  }

  public void goUp(){
    //TODO make this shuffleboard changeable
    climberLeader.set(ControlMode.PercentOutput, .10);

  }
 
  public void goDown() {
    climberLeader.set(ControlMode.PercentOutput, -.10);
  }

    public void setOpenLoop(double percentage) {
        climberLeader.set(ControlMode.PercentOutput, percentage);
    }

    public void setOpenLoop(double percentage, double deadband) {
      //  percentage = Util.applyDeadband(percentage, Constants.CLIMBER_JOG_DEADBAND); We'll worry about deadband here later. Besides, it makes more sense 
      //to use the built-in falon motor deadbands
        setOpenLoop(percentage);
    }

    public void setMotionMagicPosition(double position) {
        climberLeader.set(ControlMode.MotionMagic, position);
    }

    public boolean isMotionMagicDone() {
      //  return Math.abs(climberLeader.getClosedLoopTarget() - this.getEncoderPos()) <= TOLERANCE;
      //motion magic is a little too much for this, let's focus on this later
      return true;

    }

    public int getEncoderPos() {
        return (int) climberLeader.getSelectedSensorPosition();
    }

    public double getEncoderVel() {
        return climberLeader.getSelectedSensorVelocity();
    }

    public double getMasterVoltage() {
        return climberLeader.getMotorOutputVoltage();
    }

    public double getFollowerVoltage() {
        return climberFollower.getMotorOutputVoltage();
    }

    public double getPercentOutput() {
        return climberLeader.getMotorOutputPercent();
    }

    public double getMasterCurrent() {
        return climberLeader.getOutputCurrent();
    }

    public void zeroSensors() {
        climberLeader.setSelectedSensorPosition(0);
    }

    public boolean isFwdLimitTripped() {
        return climberLeader.getSensorCollection().isFwdLimitSwitchClosed() != 0;
    }

    public boolean isRevLimitTripped() {
        return climberLeader.getSensorCollection().isRevLimitSwitchClosed() != 0;
    }

    public void stop() {
        climberLeader.stopMotor();
    }
}
