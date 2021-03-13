/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.VelocityPeriod;
import com.ctre.phoenix.motorcontrol.*;
import frc.robot.RobotContainer;
import frc.robot.util.Util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  private WPI_TalonSRX leftMotor =  new WPI_TalonSRX(Constants.kShooterTalonMotorPort); //shooter
  private WPI_TalonSRX followMotor;
  private double shootergearRatio = 1.5;
  private WPI_TalonSRX kickUp = new WPI_TalonSRX(  Constants.kShooterTalonMotorKickUpPort); //TODO confirm this

  private CANSparkMax hood_motor = new CANSparkMax(Constants.kShooterSparkMotorHoodPort, MotorType.kBrushless);
  private CANEncoder hoodEncoder = hood_motor.getEncoder();
  private CANPIDController hoodPidController = hood_motor.getPIDController();

  private int maxDegrees = 80; //set this later
  private double hoodPositionTicks = 0;
  private boolean hoodReset = false;

  private double zeroPosition;

  private CANDigitalInput hoodLimitSwitch = hood_motor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

  //3:2 (.666)  from falcon shooter motor to shooter 
  
  // hooder motor 25:1 (25 reduction)
  //2.91 counts per 1 degree
  
  //TODO set the device ID of the sparkmax controller!!!!! 
  
  //confirm with team for limit switch on hood
  
  
  /**
  * Creates a new motor1.
  */
  public Shooter() {
    leftMotor.configFactoryDefault();
    leftMotor.setSensorPhase(true);
    leftMotor.setNeutralMode(NeutralMode.Coast);
    followMotor.follow(leftMotor);
    followMotor.setNeutralMode(NeutralMode.Coast);
    followMotor.setInverted(true);
    hood_motor.setIdleMode(IdleMode.kBrake);

    hoodEncoder.setPositionConversionFactor(42);
    
   
    configureHood();
    //shooterConfiguration(0,0,0,0); TODO set this later
  }
  
  private void configureHood() {
    // Modify constants later
    hoodPidController.setP(0);
    hoodPidController.setI(0);
    hoodPidController.setD(0);
    hoodPidController.setIZone(0);
    hoodPidController.setFF(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
    
    if (hoodLimitSwitch.get() == true){
      hood_motor.set(0);
    } 

    if(!hoodReset) {
      resetHood();
    }
  }
  
  public void setVelocity(double velocity){
    leftMotor.set(ControlMode.Velocity, velocity);
  }
  public void setpower(double power){
    leftMotor.set(ControlMode.PercentOutput, power);
  }
  
  
  public void shooterConfiguration(int kSlotIdx, int kPIDLoopIdx, int kTimeoutMs, double kP, double kI, double kD, double kF){
    /* Config sensor used for Primary PID [Velocity] */
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,kPIDLoopIdx, kTimeoutMs);
    leftMotor.setSensorPhase(true);
    
    /* Config the peak and nominal outputs */
    leftMotor.configNominalOutputForward(0, kTimeoutMs);
    leftMotor.configNominalOutputReverse(0, kTimeoutMs);
    leftMotor.configPeakOutputForward(1, kTimeoutMs);
    leftMotor.configPeakOutputReverse(-1, kTimeoutMs);
    
    
    /* Config the Velocity closed loop gains in slot0 */
    leftMotor.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
    leftMotor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
    leftMotor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
    leftMotor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
  }
  
  
  public double getShooterRPM(){ //need to convert this to RPM 
    // System.out.println(""+leftMotor.getSelectedSensorPosition());
    
    double ticksPerWheelRevolution = Constants.ENCODER_TICKS_PER_REVOLUTION_TALON * shootergearRatio;
    double sensorVelocity = leftMotor.getSelectedSensorVelocity() * 600    / ticksPerWheelRevolution ;
    return sensorVelocity;
    
  }
  
  public double getHoodAngle() {
    
    
    return -1.0;
  }
  
  
  public void changeHoodAngle(double targetAngle) {

    double targetTicks = Util.ticksToMoveHood(targetAngle);
    double deltaTicks = targetTicks - hoodPositionTicks; 

  }

  public void resetHood() {
    if(!hoodLimitSwitch.get()) {
      hood_motor.set(-0.3);
    } else {
      // reset encoders
      hood_motor.set(0);
      hoodReset = true;
     
      zeroPosition = hoodEncoder.getPosition();
      hoodPositionTicks = 0;
    }
  }
  
  public void runKickup() {
    kickUp.set(ControlMode.PercentOutput, .40); //TODO configure this
  }
  
  public void stopKickup() {
    kickUp.set(ControlMode.PercentOutput, 0);
  }
}