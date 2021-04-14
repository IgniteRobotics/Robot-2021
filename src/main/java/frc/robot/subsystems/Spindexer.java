/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Spindexer extends SubsystemBase {
    private final WPI_TalonSRX spindexerMotor;

    public double spindexerSpeed = 0.5;

    private final int TOLERANCE = 100;
    private final int CRUISE_VELOCITY = 12000;
    private final int MAX_ACCELERATION = 8000;

    /**
     * Creates a new Spindexer.
     */
    public Spindexer() {
        spindexerMotor = new WPI_TalonSRX(99);
        spindexerMotor.setInverted(false);
        spindexerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        spindexerMotor.setNeutralMode(NeutralMode.Brake);
        spindexerMotor.setSensorPhase(false);

        spindexerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
        spindexerMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10, 20);

        spindexerMotor.configMotionCruiseVelocity(CRUISE_VELOCITY, 10);
        spindexerMotor.configMotionAcceleration(MAX_ACCELERATION, 10);
    }

    public void spinClockwise(double speed) {
        spindexerMotor.set(ControlMode.PercentOutput, speed);
    }

    public void spinCounterClockwise(double speed) {
        spindexerMotor.set(ControlMode.PercentOutput, -speed);
    }

    public double getEncoderPosition() {
        return spindexerMotor.getSelectedSensorPosition();
    }

    public void stop() {
        spindexerMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void resetEncoder() {
        spindexerMotor.setSelectedSensorPosition(0);
    }

    public void configPIDProfile(int profile, double fVal, double pVal, double iVal, double dVal) {
        spindexerMotor.selectProfileSlot(profile, 0);
        spindexerMotor.config_kF(0, fVal, 20);
        spindexerMotor.config_kP(0, pVal, 20);
        spindexerMotor.config_kI(0, iVal, 20);
        spindexerMotor.config_kD(0, dVal, 20);
        spindexerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, profile, 20);
    }

    public void moveToPosition(int ticks) {
        spindexerMotor.set(ControlMode.Position, ticks);
    }

    public void setMotionMagicPosition(double position) {
        spindexerMotor.set(ControlMode.MotionMagic, position);
    }

    public boolean isMotionMagicDone() {
        return Math.abs(spindexerMotor.getClosedLoopTarget() - this.getEncoderPosition()) <= TOLERANCE;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Spindexer Encoder Pos", getEncoderPosition());
    }
}
