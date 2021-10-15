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
    private double ticksPerMeter = (2048.0 * 7.8) / Constants.WHEEL_CIRCUMFERENCE_METERS;

    public RamseteDriveSubsystem() {
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0)); // assume robot starts at x =0, y=0,
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

        // Falcons don't need to set sensor phase
        // rightMaster.setSensorPhase(false);
        // rightMaster.setInverted(false);
        // rightFollower.setInverted(false);
        // leftMaster.setSensorPhase(true);

        // uninvert right
        leftMaster.setInverted(false);
        leftFollower.setInverted(false);

        rightMaster.overrideLimitSwitchesEnable(false);
        leftMaster.overrideLimitSwitchesEnable(false);

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        // inversion etc has to happen BEFORE this statement!
        m_driveTrain = new DifferentialDrive(leftMaster, rightMaster);
        SmartDashboard.putData("Field", m_field);

        // Testing
        // m_driveTrain.setMaxOutput(.5);
        addChild("LeftMaster- Drivetrain", leftMaster);
        addChild("rightMaster- Drivetrain", rightMaster);
        addChild("rightFollower- Drivetrain", rightFollower);
        addChild("leftFollower- Drivetrain", leftFollower);
        addChild("navX", navX);

    }

    private void enableEncoders() {
        encodersAvailable = leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                10) == ErrorCode.OK
                & rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10) == ErrorCode.OK;
        if (!encodersAvailable) {
            DriverStation.reportError("Failed to configure drivetrain encoders!", false);
            useEncoders = false;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(Rotation2d.fromDegrees(getHeading()),
                Util.getMetersFromEncoderTicks(getLeftEncoderPosition()),
                Util.getMetersFromEncoderTicks(getRightEncoderPosition()));
        m_field.setRobotPose(m_odometry.getPoseMeters());

        //outputTelemetry();
    }

    public Pose2d getCurrentPose() {
        return m_odometry.getPoseMeters();
    }

    public void saveCurrentPose() {
        savedPose = getCurrentPose();
    }

    public Pose2d getSavedPose() {
        return savedPose;
    }

    public double getWheelSpeedsMetersPerSecond(double ticksPer100ms) {
        // CTRE encoders return raw sensor units per 100 miliseconds

        // Ticks / 100second
        // Wheel circumference / 2048
        return ticksPer100ms * 10 * (1 / ticksPerMeter);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {

        // TODO Convert this to wheel speeds later
        // DifferentialDriveWHeelSpeeds expects meters per second
        return new DifferentialDriveWheelSpeeds(getWheelSpeedsMetersPerSecond(leftMaster.getSelectedSensorVelocity()),
                getWheelSpeedsMetersPerSecond(rightMaster.getSelectedSensorVelocity()));
    }

    public void resetOdometry() {
        resetEncoders();
        this.zeroHeading();
        savedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        m_odometry.resetPosition(savedPose, Rotation2d.fromDegrees(getAngle()));
    }

    public void resetOdometry(Pose2d startingPose) {
        resetEncoders();
        this.zeroHeading();
        m_odometry.resetPosition(startingPose, Rotation2d.fromDegrees(getAngle()));
    }

    public void arcadeDrive(final double speed, final double rotation, final boolean useSquares) {
        var xSpeed = speedRateLimiter.calculate(safeClamp(speed));
        var zRotation = rotationRateLimiter.calculate(safeClamp(rotation));
        if (useSquares) {
            // xSpeed *= Math.abs(xSpeed);
            // zRotation *= Math.abs(zRotation);
            xSpeed = Util.applyLimiters(xSpeed, velocityRampExponent, velocityLimitMultiplier);
            zRotation = Util.applyLimiters(zRotation, turnRampExponent, turnLimitMultiplier);
        }
        // xSpeed *= Constants.kMaxSpeedMetersPerSecond;
        // zRotation *= Constants.kMaxAngularVelocity;
        m_driveTrain.arcadeDrive(xSpeed, zRotation, false);
    }

    public void tankDrivePower(double leftPower, double rightPower) {
        double leftPowerLimited = safeClamp(leftPower);
        double rightPowerLimited = safeClamp(rightPower);

        m_driveTrain.tankDrive(leftPowerLimited, rightPowerLimited, false);
    }

    public void tankDrive(final double leftSpeed, final double rightSpeed, final boolean useSquares) {
        var xLeftSpeed = safeClamp(leftSpeed) * Constants.kMaxSpeedMetersPerSecond;
        var xRightSpeed = safeClamp(rightSpeed) * Constants.kMaxSpeedMetersPerSecond;
        if (useSquares) {
            xLeftSpeed *= Math.abs(xLeftSpeed);
            xRightSpeed *= Math.abs(xRightSpeed);
        }
        if (useEncoders) {
            tankDriveVelocity(xLeftSpeed, xRightSpeed);
        } else {
            m_driveTrain.tankDrive(leftSpeed, rightSpeed, useSquares);
        }
    }

    public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
        // TODO - remove later. liting velocity to 1 m/s
        leftVelocity = Util.limit(leftVelocity, 1.0);
        rightVelocity = Util.limit(rightVelocity, 1.0);
        final double leftAccel = (leftVelocity
                - Util.stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity())) / .20;
        final double rightAccel = (rightVelocity
                - Util.stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity())) / .20;

        final double leftFeedForwardVolts = Constants.FEED_FORWARD.calculate(leftVelocity, leftAccel);
        final double rightFeedForwardVolts = Constants.FEED_FORWARD.calculate(rightVelocity, rightAccel);

        leftMaster.set(ControlMode.Velocity, Util.metersPerSecToStepsPerDecisec(leftVelocity),
                DemandType.ArbitraryFeedForward, leftFeedForwardVolts / 12);
        rightMaster.set(ControlMode.Velocity, Util.metersPerSecToStepsPerDecisec(rightVelocity),
                DemandType.ArbitraryFeedForward, rightFeedForwardVolts / 12);
        m_driveTrain.feed();
    }

    // used to drive trajectories
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.leftMaster.setVoltage(leftVolts);
        this.leftFollower.setVoltage(leftVolts);

        // if you're raw setting volts, you need to flip the right side.
        this.rightMaster.setVoltage(-rightVolts);
        this.rightFollower.setVoltage(-rightVolts);
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
        tankDriveVelocity(0, 0);
    }

    public Command createCommandForTrajectory(final Trajectory trajectory) {
        return new RamseteCommand(trajectory, this::getCurrentPose,
                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics,
                this::tankDriveVelocity, this).andThen(this::stop, this);
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

    public double getLeftMasterPositionTicks() {
        return leftMaster.getSelectedSensorPosition();
    }
}