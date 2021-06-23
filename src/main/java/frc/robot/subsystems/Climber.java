/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.Util;

public class Climber extends SubsystemBase {
    /**
     * Creates a new Climber.
     */
    private WPI_TalonFX climberLeader;
    private WPI_TalonFX climberFollower;

    private Command defaultCommand;

    private final double kF = 0;
    private final double kP = 1;
    private final double kI = 0;
    private final double kD = 0;

    private final int MAX_ACCELERATION = 8000 / 2;
    private final int CRUISE_VELOCITY = 6000;

    private final int TOLERANCE = 200;

    public Climber(int climberLeaderID, int climberFollowerID) {
        climberLeader = new WPI_TalonFX(climberLeaderID);
        climberFollower = new WPI_TalonFX(climberFollowerID);

        climberLeader.setNeutralMode(NeutralMode.Brake);
        climberFollower.setNeutralMode(NeutralMode.Brake);

        climberFollower.follow(climberLeader);

        climberLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        climberLeader.setSensorPhase(true);
        climberLeader.setInverted(false);

        climberFollower.setInverted(InvertType.FollowMaster);

        climberLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
        climberLeader.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

        climberLeader.selectProfileSlot(0, 0);
        climberLeader.config_kF(0, kF, 10);
        climberLeader.config_kP(0, kP, 10);
        climberLeader.config_kI(0, kI, 10);
        climberLeader.config_kD(0, kD, 10);

        climberLeader.configMotionCruiseVelocity(CRUISE_VELOCITY, 10);
        climberLeader.configMotionAcceleration(MAX_ACCELERATION, 10);

        //Livewindow methods for testing
        addChild("climberLeader- Climber",climberLeader);
        addChild("climberFollower- Climber",climberFollower);

        
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setOpenLoop(double percentage) {
        climberLeader.set(ControlMode.PercentOutput, percentage);
    }

    public void setOpenLoop(double percentage, double deadband) {
        percentage = Util.applyDeadband(percentage, Constants.CLIMBER_JOG_DEADBAND);
        setOpenLoop(percentage);
    }

    public void setMotionMagicPosition(double position) {
        climberLeader.set(ControlMode.MotionMagic, position);
    }

    public boolean isMotionMagicDone() {
        return Math.abs(climberLeader.getClosedLoopTarget() - this.getEncoderPos()) <= TOLERANCE;
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
