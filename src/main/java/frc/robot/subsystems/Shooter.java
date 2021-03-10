/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.VelocityPeriod;
import com.ctre.phoenix.motorcontrol.*;
import frc.robot.RobotContainer;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  
  //private int max_degrees = 360;
  //private WPI_TalonSRX left_motor =  new WPI_TalonSRX(Constants.kShooterTalonMotorPort); //shooter
  //private WPI_VictorSPX follow1;
  //private CANSparkMax hood_motor = new CANSparkMax(0, MotorType.kBrushless);
  //private CANEncoder hoodEncoder = hood_motor.getEncoder();
  //private CANPIDController hoodPidController = hood_motor.getPIDController();


  //3:2 (.666)  from falcon shooter motor to shooter 
  // hooder motor 25:1 (25 reduction)
  //2.91 counts per 1 degree
  
  //TODO set the device ID of the sparkmax controller!!!!!

  private WPI_TalonSRX right_motor;  

  //confirm with team for limit switch on hood
  
  
  /**
   * Creates a new motor1.
   */
  public Shooter() {
  
  //  follow1 = new WPI_TalonSRX(Constants.kShooterTalonMotorPort2);
   //motor.configFactoryDefault();
    //motor.setNeutralMode(NeutralMode.Coast);
    //follow1.follow(motor);
    //follow1.setNeutralMode(NeutralMode.Coast);
    //follow1.setInverted(true);
    

  }

  private void configureHood() {
    // Modify constants later
    //hoodPidController.setP(0);
    //hoodPidController.setI(0);
    //hoodPidController.setD(0);
    //hoodPidController.setIZone(0);
    //hoodPidController.setFF(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVelocity(double Velocity){
   // motor.set(ControlMode.Velocity, Velocity);
  }
  public void setpower(double power){
    //motor.set(ControlMode.PercentOutput, power);
  }
  

  public void configuration(int kSlotIdx, int kPIDLoopIdx, int kTimeoutMs, double kP, double kI, double kD, double kF){
    /* Config sensor used for Primary PID [Velocity] */
   // motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,kPIDLoopIdx, kTimeoutMs);
   // motor.setSensorPhase(true);
    
    
    
		/* Config the peak and nominal outputs */
	//	motor.configNominalOutputForward(0, kTimeoutMs);
	//	motor.configNominalOutputReverse(0, kTimeoutMs);
	//	motor.configPeakOutputForward(1, kTimeoutMs);
   // motor.configPeakOutputReverse(-1, kTimeoutMs);

    
		/* Config the Velocity closed loop gains in slot0 */
	//	motor.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
	//  motor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
	//	motor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
   // motor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
  }

  public void defaultConfig(){
   // motor.configFactoryDefault();
  //  motor.setSensorPhase(true);
        
  //  motor.setNeutralMode(NeutralMode.Coast);
    //follow1.setNeutralMode(NeutralMode.Coast);
    //follow1.setInverted(true);
    //follow2.setNeutralMode(NeutralMode.Coast);
  }

  public int getvelocity(){
 //  System.out.println(""+motor.getSelectedSensorPosition());
  // return (int) (motor.getSelectedSensorVelocity());

    return -1; //placeholder
  }

}
