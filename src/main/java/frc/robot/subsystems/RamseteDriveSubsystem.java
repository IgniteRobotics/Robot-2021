/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

import frc.robot.util.Util;

public class RamseteDriveSubsystem extends SubsystemBase {

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(Constants.kLeftMasterPort);
  private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(Constants.kLeftFollowerPort);
  private final WPI_VictorSPX leftFollower2 = new WPI_VictorSPX(Constants.kLeftFollowerPort2);

  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(Constants.kRightMasterPort);
  private final WPI_VictorSPX rightFollower = new WPI_VictorSPX(Constants.kRightFollowerPort);
  private final WPI_VictorSPX rightFollower2 = new WPI_VictorSPX(Constants.kRightFollowerPort2);

  private final DifferentialDrive m_driveTrain = new DifferentialDrive(leftMaster, rightMaster);

  private final AHRS navX = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveOdometry m_odometry;

  private Pose2d savedPose;

  private boolean useEncoders;
  private boolean encodersAvailable;

  private final SlewRateLimiter speedRateLimiter = new SlewRateLimiter(Constants.SPEED_RATE_LIMIT_ARCADE);
  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(Constants.ROTATION_RATE_LIMIT_ARCADE);

  public RamseteDriveSubsystem() {
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading())); //assume robot starts at x =0, y=0, theta = 0
    resetEncoders();
    navX.zeroYaw();

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    talonConfig.slot0.kP = Constants.kPDriveVel;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.openloopRamp = Constants.OPEN_LOOP_RAMP;

    leftMaster.configAllSettings(talonConfig);
    leftMaster.enableVoltageCompensation(true);
    leftFollower.configFactoryDefault();
    leftFollower2.configFactoryDefault();

    rightMaster.configAllSettings(talonConfig);
    rightMaster.enableVoltageCompensation(true);
    rightFollower.configFactoryDefault();
    rightFollower2.configFactoryDefault();

    enableEncoders();

    setNeutralMode(NeutralMode.Brake);

    rightMaster.setSensorPhase(false);
    rightMaster.setInverted(true);
    rightFollower.setInverted(true);
    rightFollower2.setInverted(true);

    leftMaster.setSensorPhase(false);
    leftMaster.setInverted(false);
    leftFollower.setInverted(false);
    leftFollower2.setInverted(false);

    rightMaster.overrideLimitSwitchesEnable(false);
    leftMaster.overrideLimitSwitchesEnable(false);

    leftFollower.follow(leftMaster);
    leftFollower2.follow(leftMaster);
    rightFollower.follow(rightMaster);
    rightFollower2.follow(rightMaster);
  }

  public void addDashboardWidgets(ShuffleboardLayout dashboard) {
    dashboard.addString("Pose", () -> m_odometry.getPoseMeters().toString());

    var useEncodersEntry = dashboard.addPersistent("Use Encoders", useEncoders)
        .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    useEncodersEntry.addListener(this::handleEncoderEntry, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  private void handleEncoderEntry(EntryNotification notification) {
    var entry = notification.getEntry();
    if(entry.getBoolean(true) && (!encodersAvailable || !useEncoders)) {
      useEncoders = true;
      enableEncoders();
    }
    else if(!entry.getBoolean(true)) {
      useEncoders = false;
    }
    entry.setBoolean(useEncoders);
  }

  private void enableEncoders() {
    encodersAvailable = 
      leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10) == ErrorCode.OK &
      rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10) == ErrorCode.OK;
    if(!encodersAvailable) {
      DriverStation.reportError("Failed to configure drivetrain encoders!", false);
      useEncoders = false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), Util.getMetersFromEncoderTicks(getLeftEncoderPosition()),
        Util.getMetersFromEncoderTicks(getRightEncoderPosition()));
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
    savedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0)); 
    m_odometry.resetPosition(savedPose, Rotation2d.fromDegrees(getHeading()));
  }


  //not sure if I want to use this. Might want to use the other drivetrain's arcadeDrive() - Rossy
  public void arcadeDrive(final double speed, final double rotation, final boolean useSquares) {
      var xSpeed = speedRateLimiter.calculate(safeClamp(speed));
      var zRotation = -rotationRateLimiter.calculate(safeClamp(rotation));
      if (useSquares) {
        xSpeed *= Math.abs(xSpeed);
        zRotation *= Math.abs(zRotation);
      }
      xSpeed *= Constants.kMaxSpeedMetersPerSecond;
      zRotation *= Constants.kMaxAngularVelocity;
      var wheelSpeeds = Constants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, zRotation));
      if(useEncoders) {
        tankDriveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
      }
      else {
      m_driveTrain.arcadeDrive(speed, rotation, useSquares);
    }
  }

  public void tankDrive(final double leftSpeed, final double rightSpeed, final boolean useSquares) {
      var xLeftSpeed = safeClamp(leftSpeed) * Constants.kMaxSpeedMetersPerSecond;
      var xRightSpeed = safeClamp(rightSpeed) * Constants.kMaxSpeedMetersPerSecond;
      if (useSquares) {
        xLeftSpeed *= Math.abs(xLeftSpeed);
        xRightSpeed *= Math.abs(xRightSpeed);
      }
      if(useEncoders){
        tankDriveVelocity(xLeftSpeed, xRightSpeed);
      }
      else {
      m_driveTrain.tankDrive(leftSpeed, rightSpeed, useSquares);
    }
  }

  public void tankDriveVelocity(final double leftVelocity, final double rightVelocity) {
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

  private double safeClamp(final double input) {
    if (Double.isNaN(input)) {
      return 0;
    }
    return MathUtil.clamp(input, -1, 1);
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
    return Math.IEEEremainder(navX.getAngle(), 360.0) * (1.0);
  }

  public void setNeutralMode(final NeutralMode neutralMode) {
    leftMaster.setNeutralMode(neutralMode);
    rightMaster.setNeutralMode(neutralMode);
    leftFollower.setNeutralMode(neutralMode);
    rightFollower.setNeutralMode(neutralMode);
    leftFollower2.setNeutralMode(neutralMode);
    rightFollower2.setNeutralMode(neutralMode);
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
}