/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int kLeftMasterPort = 1;
    public static final int kLeftFollowerPort = 3;
    public static final int kLeftFollowerPort2 = 5;
    public static final int kRightMasterPort = 2;
    public static final int kRightFollowerPort = 4;
    public static final int kRightFollowerPort2 = 6;

    public static final int kDriveControllerPort = 0;
    public static final int kManipControllerPort = 1;

	public static final double kDriveDeadband = 0.03;
	
	public static final int kIntakeMotorPort = 10; //TODO find out what these actually are.
	public static final int kSorterMotorPort = 11;
	public static final int kSpindexerMotorPort = 14;
	public static final int kShooterTalonMotorPort = 13;
	public static final int kShooterTalonMotorPort2 = 8;
	public static final int kKickerMotorPort = 9;
	public static final int kClimberMotorPort = 7;
	public static final int kTurretMotorPort = 12;

	public static final int kIntakeSolenoidPort = 0;
	public static final int kKickerSolenoidPort = 1;

	public static final int kSpindexerHallEffectPort = 0;
	public static final int kSorterSensorPort = 1;

    public static final int BUTTON_A = 1;
	public static final int BUTTON_B = 2;
	public static final int BUTTON_X = 3;
	public static final int BUTTON_Y = 4;
	public static final int BUTTON_LEFT_BUMPER = 5;
	public static final int BUTTON_RIGHT_BUMPER = 6;
	public static final int BUTTON_BACK = 7;
	public static final int BUTTON_START = 8;
	public static final int BUTTON_LEFT_STICK = 9;
	public static final int BUTTON_RIGHT_STICK = 10;

	public static final int AXIS_LEFT_STICK_X = 0;
	public static final int AXIS_LEFT_STICK_Y = 1;
	public static final int AXIS_LEFT_TRIGGER = 2;
	public static final int AXIS_RIGHT_TRIGGER = 3;
	public static final int AXIS_RIGHT_STICK_X = 4;
	public static final int AXIS_RIGHT_STICK_Y = 5;

	public static final int BUTTON_DPAD_UP = 0;
	public static final int BUTTON_DPAD_LEFT = 270;
	public static final int BUTTON_DPAD_RIGHT = 90;
	public static final int BUTTON_DPAD_DOWN = 180;

	public static final double SPEED_RATE_LIMIT_ARCADE = 1.5;
	public static final double ROTATION_RATE_LIMIT_ARCADE = 3.0;

	public static final double SLOW_MODE_SPEED_MODIFIER = 0.5;
	public static final double CLIMBER_JOG_DEADBAND = 0.1;

	public static final double ksVolts = 0.584;
	public static final double kvVoltSecondsPerMeter = 2.04;
	public static final double kaVoltSecondsSquaredPerMeter = 0.39;

	public static final double kPDriveVel = 14.4;

	public static final double kTrackwidthMeters = 0.67;
	public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

	public static final double kMaxSpeedMetersPerSecond = 1.0;
	public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
	public static final double kMaxAngularVelocity = 1.0;

	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;

	public static final double kWheelDiameterMeters = 0.1524;
	public static final double kEncoderCPR = 8192;

	public static final double kEncoderDistancePerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

	public static final boolean kGyroReversed = false;

	public static final double OPEN_LOOP_RAMP = 0.25;

	public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
}
