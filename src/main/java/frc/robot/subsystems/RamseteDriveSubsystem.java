/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;
import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import badlog.lib.BadLog;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.igniterobotics.robotbase.calc.SensorProfile;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;

import frc.robot.Main;
import frc.robot.constants.Constants;
import frc.robot.constants.MotorConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.util.Util;

public class RamseteDriveSubsystem extends SubsystemBase {

    private final WPI_TalonFX leftMaster = new WPI_TalonFX(MotorConstants.kLeftMasterPort);
    private final WPI_TalonFX leftFollower = new WPI_TalonFX(MotorConstants.kLeftFollowerPort);

    private final WPI_TalonFX rightMaster = new WPI_TalonFX(MotorConstants.kRightMasterPort);
    private final WPI_TalonFX rightFollower = new WPI_TalonFX(MotorConstants.kRightFollowerPort);

    private DifferentialDrive m_driveTrain;

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private final DifferentialDriveOdometry m_odometry;

    private Pose2d savedPose;

    private boolean useEncoders = true;
    private boolean encodersAvailable;

    private final SlewRateLimiter speedRateLimiter = new SlewRateLimiter(Constants.SPEED_RATE_LIMIT_ARCADE);
    private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(Constants.ROTATION_RATE_LIMIT_ARCADE);

    // create a field to send odometry data to.
    private Field2d m_field = new Field2d();

    private double velocityRampExponent = Constants.VELOCITY_RAMP_EXPONENT;
    private double velocityLimitMultiplier = Constants.VELOCITY_LIMIT_MULTIPLIER;
    private double turnRampExponent = Constants.TURN_RAMP_EXPONENT;
    private double turnLimitMultiplier = Constants.TURN_LIMIT_MULTIPLIER;
    // 7.8 is gear ratio
    private SensorProfile sensorProfile = new SensorProfile(2048, 7.8);
    private double ticksPerMeter = (2048.0 * 7.8) / Constants.WHEEL_CIRCUMFERENCE_METERS;

    private ReportingNumber leftEncoderPosition = new ReportingNumber("Left Encoder", ReportingLevel.COMPETITON);
    private ReportingNumber rightEncoderPosition = new ReportingNumber("Right Encoder", ReportingLevel.COMPETITON);
    private ReportingNumber leftEncoderVelocity = new ReportingNumber("Left Velocity", ReportingLevel.COMPETITON);
    private ReportingNumber rightEncoderVelocity = new ReportingNumber("Right Velocity", ReportingLevel.COMPETITON);
    private ReportingNumber theta = new ReportingNumber("NavX Angle", ReportingLevel.COMPETITON);

    public RamseteDriveSubsystem() {
        m_odometry = new DifferentialDriveOdometry(navX.getRotation2d()); // assume robot starts at x =0, y=0,
                                                                               // theta = 0

        navX.zeroYaw();

        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();

        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        talonConfig.slot0.kP = .01;
        talonConfig.slot0.kI = 0.0;
        talonConfig.slot0.kD = 0.0;
        talonConfig.slot0.integralZone = 400;
        talonConfig.slot0.closedLoopPeakOutput = 1.0;

        talonConfig.openloopRamp = Constants.OPEN_LOOP_RAMP;

        leftMaster.configAllSettings(talonConfig);
        leftMaster.enableVoltageCompensation(true);
        leftFollower.configFactoryDefault();

        rightMaster.configAllSettings(talonConfig);
        rightMaster.enableVoltageCompensation(true);
        rightFollower.configFactoryDefault();

        resetEncoders();

        setNeutralMode(NeutralMode.Brake);

        leftMaster.setInverted(false);
        leftFollower.setInverted(false);

        leftMaster.overrideLimitSwitchesEnable(false);
        rightMaster.overrideLimitSwitchesEnable(false);        

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        m_driveTrain = new DifferentialDrive(leftMaster, rightMaster);
    }

    @Override
    public void periodic() {
        m_odometry.update(navX.getRotation2d(), sensorProfile.uToRev(getLeftEncoderPosition()) * Constants.WHEEL_CIRCUMFERENCE_METERS,
                      sensorProfile.uToRev(-getRightEncoderPosition()) * Constants.WHEEL_CIRCUMFERENCE_METERS);
        leftEncoderPosition.set(getLeftEncoderPosition());              
        rightEncoderPosition.set(getRightEncoderPosition());    
        leftEncoderVelocity.set(getLeftEncoderVel());    
        rightEncoderVelocity.set(getRightEncoderVel());    
        theta.set(navX.getRotation2d().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {

        // TODO Convert this to wheel speeds later
        // DifferentialDriveWHeelSpeeds expects meters per second
        return new DifferentialDriveWheelSpeeds(sensorProfile.uToRPS(leftMaster.getSelectedSensorVelocity()) * Constants.WHEEL_CIRCUMFERENCE_METERS,
                sensorProfile.uToRPS(-rightMaster.getSelectedSensorVelocity()) * Constants.WHEEL_CIRCUMFERENCE_METERS);
    }

    public void resetOdometry() {
        resetEncoders();
        m_odometry.resetPosition(savedPose, Rotation2d.fromDegrees(getAngle()));
    }

    public void resetOdometry(Pose2d startingPose) {
        resetEncoders();
        m_odometry.resetPosition(startingPose, navX.getRotation2d());
    }

    public void arcadeDrive(final double speed, final double rotation, final boolean useSquares) {
        var xSpeed = speedRateLimiter.calculate(safeClamp(speed));
        var zRotation = rotationRateLimiter.calculate(safeClamp(rotation));
        // xSpeed *= Constants.kMaxSpeedMetersPerSecond;
        // zRotation *= Constants.kMaxAngularVelocity;
        m_driveTrain.arcadeDrive(xSpeed, zRotation, useSquares);
    }

    public void tankDrivePower(double leftPower, double rightPower) {
        double leftPowerLimited = safeClamp(leftPower);
        double rightPowerLimited = safeClamp(rightPower);

        m_driveTrain.tankDrive(leftPowerLimited, rightPowerLimited, false);
    }


    // used to drive trajectories
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.leftMaster.setVoltage(leftVolts);
        this.rightMaster.setVoltage(-rightVolts);
        m_driveTrain.feed();
    }

    private double safeClamp(final double input) {
        if (Double.isNaN(input)) {
            return 0;
        }
        return MathUtil.clamp(input, -1, 1);
    }

    public void driveCurvature(double xSpeed, double zRotation, boolean isQuickTurn) {
        m_driveTrain.curvatureDrive(xSpeed, zRotation, isQuickTurn);
    }

    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    public double getAverageEncoderDistance() {
        return (leftMaster.getSelectedSensorPosition(0) + rightMaster.getSelectedSensorPosition(0)) / 2.0;
    }

    public double getLeftEncoderPosition() {
        return leftMaster.getSelectedSensorPosition(0);
    }

    public double getRightEncoderPosition() {
        return rightMaster.getSelectedSensorPosition(0);
    }

    public void setMaxOutput(final double maxOutput) {
        m_driveTrain.setMaxOutput(maxOutput);
    }

    public void zeroHeading() {
        navX.reset();
    }

    /**
     * Returns the absolute position of the yaw axis.
     * 
     * Increments over 360
     * @return
     */
    public double getHeading() {
        return navX.getAngle();
    }

    public void setNeutralMode(final NeutralMode neutralMode) {
        leftMaster.setNeutralMode(neutralMode);
        rightMaster.setNeutralMode(neutralMode);
        leftFollower.setNeutralMode(neutralMode);
        rightFollower.setNeutralMode(neutralMode);
    }

    public void stop() {
        tankDriveVolts(0, 0);
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
        return rightMaster.getStatorCurrent();
    }

    public boolean isConnected() {
        return navX.isConnected();
    }

    public double getAngle() {
        return -navX.getAngle();
    }

    public double getYaw() {
        return -navX.getYaw();
    }

    public double getClosedLoopTarget() {
        return leftMaster.getClosedLoopTarget();
    }

    // TODO: Fix this to be more consistent with inverted and negative voltages.
    // Right now it's sort of a hack
    public void driveDistance(double setpointTicks) {
        // I don't think the arbitary feed forward is really that helpful here
        leftMaster.set(TalonFXControlMode.Position, setpointTicks, DemandType.ArbitraryFeedForward, 0.0007);
        rightMaster.set(TalonFXControlMode.Position, -setpointTicks, DemandType.ArbitraryFeedForward, 0.0007);
    }
}