/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

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

    private ShuffleboardTab tab;
    private NetworkTableEntry velocityRampExponentEntry;
    private NetworkTableEntry velocityLimitMultiplierEntry;
    private NetworkTableEntry turnRampExponentEntry;
    private NetworkTableEntry turnLimitMultiplierEntry;

    private double velocityRampExponent;
    private double velocityLimitMultiplier;
    private double turnRampExponent;
    private double turnLimitMultiplier;


    public RamseteDriveSubsystem() {
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0)); //assume robot starts at x =0, y=0, theta = 0
        resetEncoders();
        navX.zeroYaw();

        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();




        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        talonConfig.slot0.kP = Constants.kPDriveVel;
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
  
        enableEncoders();

        setNeutralMode(NeutralMode.Brake);
        
        //Falcons don't need to set sensor phase
        //rightMaster.setSensorPhase(false);
        //rightMaster.setInverted(false);
        //rightFollower.setInverted(false);
       // leftMaster.setSensorPhase(true);

//uninvert right
        leftMaster.setInverted(false);
        leftFollower.setInverted(false);

        rightMaster.overrideLimitSwitchesEnable(false);
        leftMaster.overrideLimitSwitchesEnable(false);

        leftFollower.follow(leftMaster);
        rightFollower.follow(rightMaster);

        //inversion etc has to happen BEFORE this statement!
        m_driveTrain = new DifferentialDrive(leftMaster, rightMaster);
        SmartDashboard.putData("Field", m_field);

        tab = Shuffleboard.getTab("Drivetrain");
        velocityRampExponentEntry = tab.add("DT Vel Ramp Exp", Constants.VELOCITY_RAMP_EXPONENT).withProperties(Map.of("min", 1)).getEntry();
        velocityLimitMultiplierEntry = tab.add("DT Vel Lim Mult", Constants.VELOCITY_LIMIT_MULTIPLIER).withProperties(Map.of("min", 0.1, "max", 1)).getEntry();
        turnRampExponentEntry = tab.add("DT Turn Ramp Exp", Constants.TURN_RAMP_EXPONENT).withProperties(Map.of("min", 1)).getEntry();
        turnLimitMultiplierEntry = tab.add("DT Trun Lim Mult", Constants.TURN_LIMIT_MULTIPLIER).withProperties(Map.of("min", 0.1, "max", 1)).getEntry();

        velocityLimitMultiplier = velocityLimitMultiplierEntry.getDouble(Constants.VELOCITY_LIMIT_MULTIPLIER);
        velocityRampExponent = velocityRampExponentEntry.getDouble(Constants.VELOCITY_RAMP_EXPONENT);
        turnRampExponent = turnRampExponentEntry.getDouble(Constants.TURN_RAMP_EXPONENT);
        turnLimitMultiplier = turnLimitMultiplierEntry.getDouble(Constants.TURN_LIMIT_MULTIPLIER);

    }

    private void enableEncoders() {
        encodersAvailable =
                leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10) == ErrorCode.OK &
                        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10) == ErrorCode.OK;
        if (!encodersAvailable) {
            DriverStation.reportError("Failed to configure drivetrain encoders!", false);
            useEncoders = false;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), Util.getMetersFromEncoderTicks(getLeftEncoderPosition()),
                Util.getMetersFromEncoderTicks(getRightEncoderPosition()));
        m_field.setRobotPose(m_odometry.getPoseMeters());

        velocityLimitMultiplier = velocityLimitMultiplierEntry.getDouble(Constants.VELOCITY_LIMIT_MULTIPLIER);
        velocityRampExponent = velocityRampExponentEntry.getDouble(Constants.VELOCITY_RAMP_EXPONENT);
        turnRampExponent = turnRampExponentEntry.getDouble(Constants.TURN_RAMP_EXPONENT);
        turnLimitMultiplier = turnLimitMultiplierEntry.getDouble(Constants.TURN_LIMIT_MULTIPLIER);

        outputTelemetry();
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

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity(),
                rightMaster.getSelectedSensorVelocity());
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
        //TODO - remove later.  liting velocity to 1 m/s
        leftVelocity = Util.limit(leftVelocity, 1.0);
        rightVelocity = Util.limit(rightVelocity, 1.0);
        final double leftAccel = (leftVelocity - Util.stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity()))
                / .20;
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

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        this.leftMaster.setVoltage(leftVolts);
        this.leftFollower.setVoltage(leftVolts);

        //if you're raw setting volts, you need to flip the right side.
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

    public double getHeading() {
        return Math.IEEEremainder(-navX.getAngle(), 360.0) * (1.0);
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
        return new RamseteCommand(
                trajectory,
                this::getCurrentPose,
                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                Constants.kDriveKinematics,
                this::tankDriveVelocity,
                this)
                .andThen(this::stop, this);
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


    public void outputTelemetry() {
        SmartDashboard.putNumber("Drivetrain/Left enc pos", this.getLeftEncoderPosition());
        SmartDashboard.putNumber("Drivetrain/Right enc pos", this.getRightEncoderPosition());
        SmartDashboard.putNumber("Drivetrain/Left enc vel", this.getLeftEncoderVel());
        SmartDashboard.putNumber("Drivetrain/Right enc vel", this.getRightEncoderVel());
        SmartDashboard.putNumber("Drivetrain/Left master voltage", this.getLeftMasterVoltage());
        SmartDashboard.putNumber("Drivetrain/Right master voltage", this.getRightMasterVoltage());
        SmartDashboard.putNumber("Drivetrain/Left master current", this.getLeftMasterCurrent());
        SmartDashboard.putNumber("Drivetrain/Right master current", this.getRightMasterCurrent());
        SmartDashboard.putNumber("Drivetrain/Left percent out", this.getLeftPercentOutput());
        SmartDashboard.putNumber("Drivetrain/Right percent out", this.getRightPercentOutput());
        SmartDashboard.putBoolean("Drivetrain/Is navX connected?", this.isConnected());
        SmartDashboard.putNumber("Drivetrain/Angle", this.getAngle());
        SmartDashboard.putNumber("Drivetrain/Yaw", this.getYaw());
        SmartDashboard.putNumber("Drivetrain/Closed loop target", this.getClosedLoopTarget());
        SmartDashboard.putBoolean("Drivetrain/Using Encoders?", this.useEncoders);
        SmartDashboard.putNumber("Drivetrain/Current VRamp Exp", this.velocityRampExponent);
        SmartDashboard.putNumber("Drivetrain/Current VRamp Lim", this.velocityLimitMultiplier);
        SmartDashboard.putNumber("Drivetrain/Current TRamp Exp", this.turnRampExponent);
        SmartDashboard.putNumber("Drivetrain/Current TRamp Lim", this.turnLimitMultiplier);

        SmartDashboard.putNumber("Drivetrain/Yaw", this.getYaw());

        // SmartDashboard.putNumber("Drivetrain/Turn error", this.getTurnError());
        // SmartDashboard.putNumber("Drivetrain/Turn setpoint", this.getTurnSetpoint());
    }
}