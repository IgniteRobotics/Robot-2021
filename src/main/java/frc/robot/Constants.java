/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.util.PathData;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int kLeftMasterPort = 2;
    public static final int kLeftFollowerPort = 4;
	public static final int kLeftFollowerPort2 = 6;
	
    public static final int kRightMasterPort = 1;
    public static final int kRightFollowerPort = 3;
    public static final int kRightFollowerPort2 = 5;

    public static final int kDriveControllerPort = 0;
    public static final int kManipControllerPort = 1;

	public static final double kDriveDeadband = 0.03;
	
	public static final int kIntakeMotorPort = 10; //TODO find out what these actually are.
	public static final int kSorterMotorPort = 11;
	public static final int kIndexerMotorPortLeft = 7;
	public static final int kIndexerMotorPortRight = 8;
	

	//shooter ports
	public static final int kShooterTalonMotorPort = 14;
	public static final int kShooterTalonMotorKickUpPort = 12; 
	public static final int kShooterSparkMotorHoodPort = 15;
	public static final int kShooterTalonMotorFollowerPort = 13;





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

	public static final double ksVolts = 1.02;
	public static final double kvVoltSecondsPerMeter = 1.02;
	public static final double kaVoltSecondsSquaredPerMeter =  0.268;
	public static final double kG = 0;
	public static final double kCos = 0;
	
	// public static final double kPDriveVel = 14.4;
	//TODO: tune this value!!!
	public static final double kPDriveVel = 0.000181;


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

	public static int ENCODER_TICKS_PER_REVOLUTION = 8192;

	//Talon bits.
	public static int ENCODER_TICKS_PER_REVOLUTION_TALON = 8192;	
	public static int TALON_PID_LOOP_IDX = 0;
	public static int TALON_CONFIG_TIMOUT_MS = 30;
	// 1023 is output in ?? at 100% power.  20660 is velocity at 100%
	public static double TALON_DEFAULT_KF = 1023.0/20660.0;  // 0.0495ish
	public static double TALON_DEFAULT_KP = 0.1;
	public static double TALON_DEFAULT_KI = 0;
	public static double TALON_DEFAULT_KD = 0;

	public static double HOOD_DEFAULT_KP = 0.01;
	public static double HOOD_DEFAULT_KI = 0.0;
	public static double HOOD_DEFAULT_KD = 0;
    public static double HOOD_DEFAULT_RPM = 100;
	public static double HOOD_MAX_POSITION = 1600;
	public static double HOOD_SET_POINT_DISTANCE = 4.572;

	public static double SHOOTER_GEAR_RATIO = 1.5;
	public static int SHOOTER_MAX_RPM = 6000;
	public static double SHOOTER_DEFAULT_RPM = 1250;
	
	public static int WHEEL_DIAMETER = 6; //in inches
	public static double WHEEL_DIAMETER_METERS = 0.1524;
	
	public static double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
	
	// variables for hood angle
	public static final double GEAR_TEETH_PER_DEGREE = 360.0 / 366.0;
	public static final double TICKS_PER_REVOLUTION_NEO = 42;
	public static final double HOOD_GEAR_RATIO = 25;

	//default arcade drive modifiers
	public static final double VELOCITY_RAMP_EXPONENT = 2;
	public static final double VELOCITY_LIMIT_MULTIPLIER = 1;
	public static final double TURN_RAMP_EXPONENT = 2;
	public static final double TURN_LIMIT_MULTIPLIER = 1;
	public static final double LIMELIGHT_HEIGHT = 0.61595;
	public static final double LIMELIGHT_ANGLE = 41.6; // degrees
	public static final double TARGET_HEIGHT = 2.49555;



	//Pathdata for galactic search. Units are in cm, pixel offset

	public static PathData pathARed = new PathData(0, 14.0, "28-GS-A-Red");
	public static PathData pathBRed = new PathData(0, -29, "28-GS-B-Red");

	public static PathData pathABlue = new PathData(0, 37, "28-GS-A-Blue");
	public static PathData pathBBlue = new PathData(0, 25, "28-GS-B-Blue");
	public static Trajectory robotDeterminedTrajectory; 


}
