/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveTrain extends SubsystemBase {

  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;

  boolean isSlowMode = false;

  private PIDController turnController;

  private AHRS navX;

  private Command defaultCommand;   
  private final double kP_TURN = 0.007;
  private final double kI_TURN = 0;
  private final double kD_TURN = 0;

  private final double kP_DRIVE = 1;
  private final double kI_DRIVE = 0;
  private final double kD_DRIVE = 0;
  private final double kF_DRIVE = 0;

  private final int CRUISE_VELOCITY = 2000;
  private final int MAX_ACCELERATION = 1000;

  private final double TURN_TOLERANCE = 2.0f;
  private final double DRIVE_TOLERANCE = 100.0;

  private final double MIN_TURN_POWER = 0.35;
  

  private double rotateToAngleRate;


  /**
   * Creates a new MyDriveTrain.
   */
  public DriveTrain(int leftMasterID, int leftFollowerID, int leftFollower2ID, int rightMasterID, int rightFollowerID, int rightFollower2ID) {
    leftMaster = new WPI_TalonSRX(leftMasterID);
    leftFollower = new WPI_VictorSPX(leftFollowerID);
    rightMaster = new WPI_TalonSRX(rightMasterID);
    rightFollower = new WPI_VictorSPX(rightFollowerID);

    navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);

    leftFollower.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);

    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    leftMaster.setSensorPhase(true);
    rightMaster.setSensorPhase(true);

    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    leftFollower.setInverted(InvertType.FollowMaster);
    rightFollower.setInverted(InvertType.FollowMaster);

    rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

    leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

    leftMaster.selectProfileSlot(2, 0);
    leftMaster.config_kF(2, kF_DRIVE, 10);
    leftMaster.config_kP(2, kP_DRIVE, 10);
    leftMaster.config_kI(2, kI_DRIVE, 10);
    leftMaster.config_kD(2, kD_DRIVE, 10);

    rightMaster.selectProfileSlot(1, 0);
    rightMaster.config_kF(1, kF_DRIVE, 10);
    rightMaster.config_kP(1, kP_DRIVE, 10);
    rightMaster.config_kI(1, kI_DRIVE, 10);
    rightMaster.config_kD(1, kD_DRIVE, 10);

    leftMaster.configMotionCruiseVelocity(CRUISE_VELOCITY, 10);
    leftMaster.configMotionAcceleration(MAX_ACCELERATION, 10);

    rightMaster.configMotionCruiseVelocity(CRUISE_VELOCITY, 10);
    rightMaster.configMotionAcceleration(MAX_ACCELERATION, 10);

    //turnController = new PIDController(kP_TURN, kI_TURN, kD_TURN, navX, this);

    turnController = new PIDController(kP_TURN, kI_TURN, kD_TURN);

    // TODO: Fix when Implementing turn to angle!
    // https://docs.wpilib.org/en/latest/docs/software/advanced-controls/controllers/pidcontroller.html#setting-continuous-input
    turnController.enableContinuousInput(-180.0f, 180.0f);
    // turnController.setOutputRange(-1.0, 1.0);
    // turnController.setAbsoluteTolerance(TURN_TOLERANCE);
    // turnController.setContinuous(true);

    leftMaster.configOpenloopRamp(0.15);
    rightMaster.configOpenloopRamp(0.15);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double throttle, double rotation, double deadband){
    throttle = limit(throttle); 
    //throttle = Util.applyDeadband(throttle, deadband) add util later
    rotation = limit(-rotation); // why negative?
    //throttle = Util.applyDeadband(rotation, deadband);
    throttle = Math.copySign(throttle * throttle, throttle);
    rotation = Math.copySign(rotation * rotation, rotation);

    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(rotation)), throttle);

    if (throttle >= 0.0) {
      // First quadrant, else second quadrant
      if (rotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = throttle - rotation;
      } else {
        leftMotorOutput = throttle + rotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (rotation >= 0.0) {
        leftMotorOutput = throttle + rotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = throttle - rotation;
      }
    }

    setOpenLoopLeft(limit(leftMotorOutput));
    setOpenLoopRight(limit(rightMotorOutput));

  }

  public int getLeftEncoderPos() {
    return leftMaster.getSelectedSensorPosition();
  }

  public int getRightEncoderPos() {
    return rightMaster.getSelectedSensorPosition();
  }

  public double getLeftEncoderVel() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public double getRightEncoderVel() {
    return rightMaster.getSelectedSensorVelocity();
  }

  public double getLeftMasterVoltage() {
    return leftMaster.getMotorOutputVoltage();
  }

  public double getRightMasterVoltage() {
    return rightMaster.getMotorOutputVoltage();
  }

  public double getLeftFollowerVoltage() {
    return leftFollower.getMotorOutputVoltage();
  }

  public double getRightFollowerVoltage() {
    return rightFollower.getMotorOutputVoltage();
  }

  public double getLeftPercentOutput() {
    return leftMaster.getMotorOutputPercent();
  }

  public double getRightPercentOutput() {
    return rightMaster.getMotorOutputPercent();
  }

  public double getLeftMasterCurrent() {
    return leftMaster.getStatorCurrent();
  }

  public double getRightMasterCurrent() {
    return rightMaster. getStatorCurrent();
  }


  public void setMotionMagicPosition(double position_inches) {
    double ticks = Util.getEncoderTicksFromInches(position_inches);
    leftMaster.set(ControlMode.MotionMagic, ticks);
    rightMaster.set(ControlMode.MotionMagic, ticks);
  }

  public boolean isMotionMagicDone() {
    return Math.abs(this.getClosedLoopTarget() - this.getLeftEncoderPos()) < DRIVE_TOLERANCE;
  }

  public double getClosedLoopTarget() {
    return leftMaster.getClosedLoopTarget();
  }



  public void zeroSensors() {
    zeroAngle();
    zeroEncoders();
  }

  public void stop() {
    leftMaster.stopMotor();
    rightMaster.stopMotor();
  }

  public double getAngle() {
    return navX.getAngle();
  }

  public double getYaw() {
    return navX.getYaw();
  }

  public void zeroAngle() {
    navX.reset();
  }

  public void zeroEncoders() {
    rightMaster.setSelectedSensorPosition(0);
    leftMaster.setSelectedSensorPosition(0);
  }

  public boolean isConnected() {
    return navX.isConnected();
  }

  public double limit(double value) {
    if (value > 1.0){
      return 1.0;
    }
    if (value < -1.0) {
      return -1.0;
    }
    return value; 
  
  }

  public void setOpenLoopLeft(double power){ //run motors
    leftMaster.set(ControlMode.PercentOutput, power);
  }
  
  public void setOpenLoopRight(double power) {
    rightMaster.set(ControlMode.PercentOutput, power);
  }

  public void toggleSlowMode() {
    isSlowMode = !(isSlowMode);
  }

    isSlowMode = !(isSlowMode);
  }

}
