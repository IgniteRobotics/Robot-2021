/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.*;

import java.util.Map;

import com.ctre.phoenix.VelocityPeriod;
import com.ctre.phoenix.motorcontrol.*;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ResetHood;
import frc.robot.util.Util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.constants.Constants;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.constants.MotorConstants;

public class Shooter extends SubsystemBase {
    private WPI_TalonFX leftMotor = new WPI_TalonFX(MotorConstants.kShooterTalonMotorPort); //shooter
    private WPI_TalonFX followMotor = new WPI_TalonFX(MotorConstants.kShooterTalonMotorFollowerPort);
    private WPI_TalonSRX kickUp = new WPI_TalonSRX(MotorConstants.kShooterTalonMotorKickUpPort); //TODO confirm this

    private CANSparkMax hood_motor = new CANSparkMax(MotorConstants.kShooterSparkMotorHoodPort, MotorType.kBrushless);
    private CANEncoder hoodEncoder = hood_motor.getEncoder();
    private CANPIDController hoodPidController = hood_motor.getPIDController();
    private CANDigitalInput hoodLimitSwitch = hood_motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    private int maxDegrees = 60;
    private double hoodPositionTicksSetPoint = 0;
    //TODO fix limit switch for hood reset
    private boolean hoodReset = false;
    private boolean extended;
    private double zeroPosition;

    private double flywheel_kP_value;
    private double flywheel_kI_value;
    private double flywheel_kD_value;

    private double hood_kP_value;
    private double hood_kI_value;
    private double hood_kD_value;
    private double hood_max_vel_value;
    private double hood_max_position_value;
    private double hood_min_position_value;

    public Shooter() {
        flywheel_kP_value = Constants.TALON_DEFAULT_KP;
        flywheel_kI_value = Constants.TALON_DEFAULT_KI;
        flywheel_kD_value = Constants.TALON_DEFAULT_KD;

        hood_kP_value = Constants.HOOD_DEFAULT_KP;
        hood_kI_value = Constants.HOOD_DEFAULT_KI;
        hood_kD_value = Constants.HOOD_DEFAULT_KD;
        hood_max_vel_value = Constants.HOOD_DEFAULT_RPM;
        hood_max_position_value = Constants.HOOD_MAX_POSITION;
        hood_min_position_value = 0;

        configureFlywheel(flywheel_kP_value, flywheel_kI_value, flywheel_kD_value);

        hood_motor.setIdleMode(IdleMode.kBrake);

        //I feel like this is wrong, and we should confirm afterwards. Luckily, this shouldnt hurt us for competiton too much
        hoodEncoder.setPositionConversionFactor(42);
        hoodEncoder.setVelocityConversionFactor(42);
        leftMotor.enableVoltageCompensation(true);
        followMotor.enableVoltageCompensation(true);

        kickUp.setInverted(true);
        kickUp.configOpenloopRamp(Constants.OPEN_LOOP_RAMP);

        configureHood(hood_kP_value, hood_kI_value, hood_kD_value, hood_max_vel_value);
    }

    private void configureFlywheel(double kP, double kI, double kD) {
        this.leftMotor.configFactoryDefault();
        this.leftMotor.setInverted(true);
        this.leftMotor.setNeutralMode(NeutralMode.Coast);

        this.leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.TALON_PID_LOOP_IDX,
                Constants.TALON_CONFIG_TIMOUT_MS);

        /* Config the peak and nominal outputs */
        this.leftMotor.configNominalOutputForward(0, 10);
        this.leftMotor.configNominalOutputReverse(0, 10);
        this.leftMotor.configPeakOutputForward(1, 10);
        this.leftMotor.configPeakOutputReverse(-1, 10);


        /* Config the Velocity closed loop gains in slot0 */
        this.leftMotor.config_kF(Constants.TALON_PID_LOOP_IDX, Constants.TALON_DEFAULT_KF, 10);
        this.leftMotor.config_kP(Constants.TALON_PID_LOOP_IDX, kP, 10);
        this.leftMotor.config_kI(Constants.TALON_PID_LOOP_IDX, kI, 10);
        this.leftMotor.config_kD(Constants.TALON_PID_LOOP_IDX, kD, 10);


        this.followMotor.follow(leftMotor);
        this.followMotor.setNeutralMode(NeutralMode.Coast);
        this.followMotor.setInverted(false);
    }

    private void configureHood(double kP, double kI, double kD, double maxV) {
        // Modify constants later
        hoodPidController.setP(kP);
        hoodPidController.setI(kI);
        hoodPidController.setD(kD);
        hoodPidController.setIZone(0);
        //hoodPidController.setFF(0.000156);

        hoodPidController.setOutputRange(-.5, .5);


        // hoodPidController.setSmartMotionMaxVelocity(maxV,0);
        // hoodPidController.setSmartMotionMinOutputVelocity(0,0);
        // hoodPidController.setSmartMotionMaxAccel(200, 0);

        hood_motor.setIdleMode(IdleMode.kBrake);


        //Feels wrong. This only feels like it matters if we care about the position of the hood in like angles.
        //For now, while it isn't ideal to deal with only ticks for setting angles, it should work
        hoodEncoder.setPositionConversionFactor(42);
        hoodEncoder.setVelocityConversionFactor(42);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter RPM", this.getShooterRPM());
        SmartDashboard.putNumber("Hood Postiton Ticks", this.getHoodTicks());
        SmartDashboard.putNumber("Hood Motor Output", this.hood_motor.getAppliedOutput());
        SmartDashboard.putBoolean("Hood in Position", this.isHoodReady());
    }

    public void setVelocity(double velocity) {
        //Don't forget this is in sensor units per 100 miliseconds!!!
        //For internal use only. Use setRPM() instead
        leftMotor.set(ControlMode.Velocity, velocity);
    }

    public void setpower(double power) {
        //limit to 1.0  do not let run backwards.
        if (power > 1.0) {
            power = 1.0;
        } else if (power < 0) {
            power = 0;
        }
        leftMotor.set(ControlMode.PercentOutput, power);
    }

    public void setRPM(int rpm) {
        if (rpm > Constants.SHOOTER_MAX_RPM) {
            rpm = Constants.SHOOTER_MAX_RPM;
        } else if (rpm < 0) {
            rpm = 0;
        }
        this.setVelocity(Util.ticksFromRPM(rpm));
    }

    public void shooterConfiguration(int kSlotIdx, int kPIDLoopIdx, int kTimeoutMs, double kP, double kI, double kD, double kF) {
        /* Config sensor used for Primary PID [Velocity] */
        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
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


    public double getShooterRPM() { //need to convert this to RPM
        // System.out.println(""+leftMotor.getSelectedSensorPosition());

        // double ticksPerWheelRevolution = Constants.ENCODER_TICKS_PER_REVOLUTION_TALON * Constants.SHOOTER_GEAR_RATIO;
        // double sensorVelocity = leftMotor.getSelectedSensorVelocity() * 600    / ticksPerWheelRevolution ;
        double rpm = Util.RPMFromTicks(leftMotor.getSelectedSensorVelocity());
        return rpm;

    }

    //Don't use this. 
    public double getHoodAngle() {
        return Util.degreesToMoveHood(getHoodTicks());
    }

    public void retractHood() {
        this.hoodPositionTicksSetPoint = this.hood_min_position_value;
        this.changeHoodTicks(this.hoodPositionTicksSetPoint);

    }

    public void extendHood() {
        this.hoodPositionTicksSetPoint = this.hood_max_position_value;
        this.changeHoodTicks(this.hoodPositionTicksSetPoint);
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
        //Change hood ticks by targetTicks amount from the CURRENT hood position
        if(targetTicks <= hood_max_position_value) {
            hoodPidController.setReference(targetTicks, ControlType.kPosition);
            this.hoodPositionTicksSetPoint = targetTicks;
        } else {
            hoodPidController.setReference(hood_max_position_value, ControlType.kPosition);
            this.hoodPositionTicksSetPoint = hood_max_position_value;
        }
 }

    public void resetHood() {
        hood_motor.set(-0.2);
    }

    public boolean isHoodReady() {
        double range = 10;
        return this.getHoodTicks() - range <= this.hoodPositionTicksSetPoint
                && this.getHoodTicks() + range >= this.hoodPositionTicksSetPoint;
    }

    public boolean isHoodReset() {
        return hoodLimitSwitch.get();
    }

    public void setHoodReset(boolean b) {
        this.hoodReset = b;
    }

    public void stopHood() {
        hood_motor.stopMotor();
    }

    public void zeroEncoders() {
        hoodEncoder.setPosition(0);
    }

    public void runKickup(double effort) {
        kickUp.set(ControlMode.PercentOutput, effort);
    }

    public void stopKickup() {
        kickUp.set(ControlMode.PercentOutput, 0);
    }

    public double getKickupPower() {
        return kickUp.get();
    }

    public void initDefaultCommand() {
        
    }
}