/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {
  
  private final double MIN_TURN_POWER = 0.35;
  private double rotateToAngleRate;
  private AHRS navX;
  private PIDController turnController;
  /**
   * Creates a new ExampleSubsystem.
   */
  private WPI_TalonSRX leftMaster;
  private WPI_VictorSPX leftFollower;
  private WPI_TalonSRX rightMaster;
  private WPI_VictorSPX rightFollower;
  public DriveTrain(int leftMasterID, int leftFollowerID, int rightMasterID, int rightFollowerID) {
  
    leftMaster = new WPI_TalonSRX(leftMasterID);
    leftFollower = new WPI_VictorSPX(leftFollowerID);
    rightMaster = new WPI_TalonSRX(rightMasterID);
    rightFollower = new WPI_VictorSPX(rightFollowerID);
    leftFollower.follow(leftMaster);
    rightFollower.follow(rightMaster);
    leftMaster.setInverted(true);
    rightMaster.setInverted(false);
    navX = new AHRS(SPI.Port.kMXP, (byte) 200);
    private double kP_TURN = SmartDashboard.getNumber(kP_TURN, 0.007);//0.007;
    private double kI_TURN = SmartDashboard.getNumber(kI_TURN, 0);
    private double kD_TURN = SmartDashboard.getNumber(kD_TURN, 0);
  
    private final double kP_DRIVE = SmartDashboard.getNumber(kP_DRIVE, 1);
    private final double kI_DRIVE = 0;
    private final double kD_DRIVE = 0;
    private final double kF_DRIVE = 0;

    turnController = new PIDController(kP_TURN, kI_TURN, kD_TURN, navX, this);

    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(TURN_TOLERANCE);
    turnController.setContinuous(true);
    

   // leftFollower.setInverted(InvertType.FollowMaster); What do these do?
   // rightFollower.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void zeroAngle() {
    navX.reset();
  }

  public void stop() {
    leftMaster.stopMotor();
    rightMaster.stopMotor();
  }
  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }

  public void enableTurnController(double setpoint) {
    turnController.setSetpoint(setpoint);
    rotateToAngleRate = 0;
    turnController.enable();
  }

  public void turnToAngle() {
    if (Math.abs(rotateToAngleRate) <= MIN_TURN_POWER) {
      this.setOpenLoopLeft(Math.copySign(MIN_TURN_POWER, rotateToAngleRate));
      this.setOpenLoopRight(-Math.copySign(MIN_TURN_POWER, rotateToAngleRate));
    } else {
      this.setOpenLoopLeft(rotateToAngleRate);
      this.setOpenLoopRight(-rotateToAngleRate);
    }
  }

  public void stopTurnController() {
    rotateToAngleRate = 0;
    turnController.disable();
    turnController.reset();
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

/*TODO */
    //driveTrain.stopTurnController();
    //driveTrain.TurnToAngle();


public void setOpenLoopLeft(double power){ //run motors
  leftMaster.set(ControlMode.PercentOutput, power);
}

public void setOpenLoopRight(double power) {
  rightMaster.set(ControlMode.PercentOutput, power);
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

}