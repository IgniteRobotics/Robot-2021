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
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Shooter extends SubsystemBase {
  private WPI_TalonFX leftMotor =  new WPI_TalonFX(Constants.kShooterTalonMotorPort); //shooter
  private WPI_TalonFX followMotor =  new WPI_TalonFX(Constants.kShooterTalonMotorFollowerPort);
  private WPI_TalonSRX kickUp = new WPI_TalonSRX(  Constants.kShooterTalonMotorKickUpPort); //TODO confirm this
  
  private CANSparkMax hood_motor = new CANSparkMax(Constants.kShooterSparkMotorHoodPort, MotorType.kBrushless);
  private CANEncoder hoodEncoder = hood_motor.getEncoder();
  private CANPIDController hoodPidController = hood_motor.getPIDController();
  
  private int maxDegrees = 80; //set this later
  private double hoodPositionTicks = 0;
  //TODO fix limit switch for hood reset
  private boolean hoodReset = true;
  private boolean extended; 
  private double zeroPosition;
  
  private CANDigitalInput hoodLimitSwitch = hood_motor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
  
  //m_pidController.setOutputRange(kMinOutput, kMaxOutput)
  
  
  
  //3:2 (.666)  from falcon shooter motor to shooter 
  
  // hooder motor 25:1 (25 reduction)
  //2.91 counts per 1 degree
  
  //TODO set the device ID of the sparkmax controller!!!!! 
  
  //confirm with team for limit switch on hood
  
  
  /**
  * Creates a new motor1.
  */
  public Shooter() {

    configureFlywheel();

    hood_motor.setIdleMode(IdleMode.kBrake);
    
    hoodEncoder.setPositionConversionFactor(42);
    
    kickUp.setInverted(true);
    
    
    configureHood();
    //shooterConfiguration(0,0,0,0); TODO set this later
  }

  private void configureFlywheel(){
    this.leftMotor.configFactoryDefault();
    this.leftMotor.setInverted(true);
    this.leftMotor.setNeutralMode(NeutralMode.Coast);

    this.leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    /* Config the peak and nominal outputs */
    this.leftMotor.configNominalOutputForward(0, 10);
    this.leftMotor.configNominalOutputReverse(0, 10);
    this.leftMotor.configPeakOutputForward(1, 10);
    this.leftMotor.configPeakOutputReverse(-1, 10);
    
    
    /* Config the Velocity closed loop gains in slot0 */
    this.leftMotor.config_kF(0, 0, 10);
    this.leftMotor.config_kP(0, 0, 10);
    this.leftMotor.config_kI(0, 0, 10);
    this.leftMotor.config_kD(0, 0, 10);


    this.followMotor.follow(leftMotor);
    this.followMotor.setNeutralMode(NeutralMode.Coast);
    this.followMotor.setInverted(false);
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
      //TODO undo this once it's tested.
      //resetHood();
    }
  }
  
  public void setVelocity(double velocity){
    leftMotor.set(ControlMode.Velocity, velocity);
  }
  public void setpower(double power){
    leftMotor.set(ControlMode.PercentOutput, power);
  }

  public void setRPM(int rpm){
    if (rpm > Constants.SHOOTER_MAX_RPM){
      rpm = Constants.SHOOTER_MAX_RPM;
    } else if (rpm < 0) {
      rpm = 0;
    }
    this.setVelocity(Util.ticksFromRPM(rpm));
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
    
    double ticksPerWheelRevolution = Constants.ENCODER_TICKS_PER_REVOLUTION_TALON * Constants.SHOOTER_GEAR_RATIO;
    double sensorVelocity = leftMotor.getSelectedSensorVelocity() * 600    / ticksPerWheelRevolution ;
    return sensorVelocity;
    
  }
  
  public double getHoodAngle() {
    return Util.degreesToMoveHood(getHoodTicks());
  }
  
  public void retractHood(){
    
  }
  
  public void extendHood() {
    
  }


  public double getHoodTicks() {
    return hoodEncoder.getPosition();
  }
  
  public void changeHoodAngle(double targetAngle) {
    // v+ hood raises
    double targetTicks = Util.ticksToMoveHood(targetAngle);
    changeHoodTicks(targetTicks);
  }
  
  public void changeHoodTicks(double targetTicks) {
    hoodPidController.setReference(targetTicks, ControlType.kPosition);
  }
  
  public void resetHood() {
    if(!hoodLimitSwitch.get()) {
      hood_motor.set(-0.3);
    } else {
      // reset encoders
      hood_motor.set(0);
      hoodReset = true;

      hoodEncoder.setPosition(0);
    }
  }
  
  public void runKickup(double effort) {
    kickUp.set(ControlMode.PercentOutput, effort); 
  }
  
  public void stopKickup() {
    kickUp.set(ControlMode.PercentOutput, 0);
  }

  public double getKickupPower(){
    return kickUp.get();
  }
}