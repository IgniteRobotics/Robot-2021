/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.commands.climber.ClimbUp;
import frc.robot.commands.climber.RetractClimbMax;
import frc.robot.commands.LimelightSnapshot;
import frc.robot.commands.ResetHeading;
import frc.robot.commands.autonomous.GalacticSearch;
import frc.robot.commands.shooter.AdjustHoodAngle;
import frc.robot.commands.shooter.ResetHood;
import frc.robot.commands.shooter.RetractHood;
import frc.robot.commands.shooter.SetHoodAngle;
import frc.robot.commands.shooter.ShootBallSpecific;
import frc.robot.commands.shooter.ShootInterpolatedBall;
import frc.robot.commands.shooter.TestExtendShooterHood;
import frc.robot.commands.shooter.TestRetractHood;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.SetIntake;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.constants.Constants;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.MotorConstants;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.drivetrain.TargetPositioning;
import frc.robot.commands.drivetrain.TargetPositioning;
import frc.robot.commands.drivetrain.TurnAngle;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.DriveTrajectory;
import frc.robot.commands.drivetrain.RamseteArcadeDrive;
import frc.robot.subsystems.RamseteDriveSubsystem;
import frc.robot.subsystems.Realsense;
import frc.robot.commands.shooter.ShootBallTest;
import frc.robot.subsystems.Climber;
import frc.robot.commands.climber.ClimbUp;
import frc.robot.commands.climber.ClimbDown;
import frc.robot.commands.climber.ClimbDownEngage;
import frc.robot.commands.climber.ClimbMotionMagicUp;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private RamseteDriveSubsystem m_driveTrain = new RamseteDriveSubsystem();
  private Intake m_intake = new Intake();
  private Shooter m_shooter = new Shooter();
  private Indexer m_indexer = new Indexer();
  private Limelight m_limelight = new Limelight();
  private Joystick m_driveController = new Joystick(MotorConstants.kDriveControllerPort);
  private Joystick m_manipController = new Joystick(MotorConstants.kManipControllerPort);
  private Climber m_climber = new Climber();
  
  SendableChooser<Command> chooseAuton = new SendableChooser<>();

  // begin current driver commands / input
  private RamseteArcadeDrive teleDriveCommand = new RamseteArcadeDrive(m_driveController, m_driveTrain);

  private ShootInterpolatedBall shootBallInterpolated = new ShootInterpolatedBall(m_shooter, m_indexer, m_limelight);
  private TargetPositioning targetingCommand = new TargetPositioning(m_driveTrain, m_driveController, m_limelight);

  private ResetHood resetHood = new ResetHood(m_shooter);
  private RetractHood retractHood = new RetractHood(m_shooter);

  private ShootBallSpecific shortShot = new ShootBallSpecific(m_shooter, m_indexer, 3500, 0);
  private ShootBallSpecific baseShot = new ShootBallSpecific(m_shooter, m_indexer, 5900, 1600);
  private ShootBallSpecific trenchShot = new ShootBallSpecific(m_shooter, m_indexer, 5500, 1600);

  private ClimbUp climbUp = new ClimbUp(m_climber);
  private ClimbDown climbDown = new ClimbDown(m_climber);
  private ClimbDownEngage climbDownEngage = new ClimbDownEngage(m_climber, m_manipController);

  private RetractClimbMax retractClimbMax = new RetractClimbMax(m_climber);

  private ToggleIntake toggleIntakeCommand = new ToggleIntake(m_intake);

  private RunIntake intakeBalls = new RunIntake(1.0, m_intake);
  private RunIntake outtakeBalls = new RunIntake(-1.0, m_intake);

  private TurnAngle turn90Degrees = new TurnAngle(m_driveTrain, 90);

  private ShootBallTest shootBallTest = new ShootBallTest(m_shooter, m_indexer);

  private SequentialCommandGroup threeBallAuton = new
  SequentialCommandGroup(
    new ResetHood(m_shooter),
    new SetIntake(m_intake, false),
    new ShootInterpolatedBall(m_shooter, m_indexer, m_limelight).withTimeout(2.5),
    new TurnAngle(m_driveTrain, 0).withTimeout(1)
  );

  private SequentialCommandGroup sixBallAuton;

  private JoystickButton btn_driverA = new JoystickButton(m_driveController, ControllerConstants.BUTTON_A);
  private JoystickButton btn_driverB = new JoystickButton(m_driveController, ControllerConstants.BUTTON_B);
  private JoystickButton btn_driverY = new JoystickButton(m_driveController, ControllerConstants.BUTTON_Y);
  private JoystickButton btn_driverX = new JoystickButton(m_driveController, ControllerConstants.BUTTON_X);
  private JoystickButton btn_driverLBumper = new JoystickButton(m_driveController,
      ControllerConstants.BUTTON_LEFT_BUMPER);
  private JoystickButton btn_driverRBumper = new JoystickButton(m_driveController,
      ControllerConstants.BUTTON_RIGHT_BUMPER);
  private JoystickButton btn_driverRStick = new JoystickButton(m_driveController,
      ControllerConstants.BUTTON_RIGHT_STICK);

  private JoystickButton btn_manipA = new JoystickButton(m_manipController, ControllerConstants.BUTTON_A);
  private JoystickButton btn_manipY = new JoystickButton(m_manipController, ControllerConstants.BUTTON_Y);
  private JoystickButton btn_manipX = new JoystickButton(m_manipController, ControllerConstants.BUTTON_X);
  private JoystickButton btn_manipB = new JoystickButton(m_manipController, ControllerConstants.BUTTON_B);
  private JoystickButton btn_manipLBumper = new JoystickButton(m_manipController,
      ControllerConstants.BUTTON_LEFT_BUMPER);
  private JoystickButton btn_manipRBumper = new JoystickButton(m_manipController,
      ControllerConstants.BUTTON_RIGHT_BUMPER);
  private JoystickButton btn_manipStart = new JoystickButton(m_manipController, ControllerConstants.BUTTON_START);

  private POVButton btn_manipPovUp = new POVButton(m_manipController, 0);
  private POVButton btn_manipPovRight = new POVButton(m_manipController, 90);
  private POVButton btn_manipPovDown = new POVButton(m_manipController, 180);
  private POVButton btn_manipPovLeft = new POVButton(m_manipController, 270);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    this.configureButtonBindings();
    this.configureSubsystemCommands();
    this.configureAutonChooser();

    SmartDashboard.putData(new ResetHeading(m_driveTrain));
  }

  private void configureButtonBindings() {
    btn_driverY.whileHeld(targetingCommand);
    btn_driverLBumper.whileHeld(outtakeBalls);
    btn_driverRBumper.whileHeld(intakeBalls);

    btn_manipLBumper.whileHeld(climbUp);
    btn_manipRBumper.whenPressed(toggleIntakeCommand);

    btn_manipA.whileHeld(shootBallInterpolated);
    btn_manipY.whileHeld(climbDown);
    btn_manipB.whileHeld(retractClimbMax);

    btn_driverRStick.whenPressed(teleDriveCommand::toggleReversed);
    
    btn_manipPovUp.whileHeld(trenchShot);
    btn_manipPovRight.whileHeld(baseShot);
    btn_manipPovDown.whileHeld(shortShot);
    btn_manipPovLeft.whileHeld(new TurnAngle(m_driveTrain, 0));
  }

  private void configureSubsystemCommands() {
    m_driveTrain.setDefaultCommand(teleDriveCommand);
    m_shooter.setDefaultCommand(retractHood);
  }

  public void configureAutonCommand() {
    Trajectory forwards = loadTrajectory("TrenchForwards");
    Trajectory backwards = loadTrajectory("TrenchBackwards");

    DriveTrajectory forwardsDrive = new DriveTrajectory(m_driveTrain, forwards);
    DriveTrajectory backwardsDrive = new DriveTrajectory(m_driveTrain, backwards);

    ParallelRaceGroup group = new ParallelRaceGroup(
      new RunIntake(1, m_intake),
      forwardsDrive
    );

    this.sixBallAuton = new SequentialCommandGroup(
      threeBallAuton,
      new ParallelCommandGroup(group, new ResetHood(m_shooter)),
      backwardsDrive,
      new TargetPositioning(m_driveTrain, m_driveController, m_limelight).withTimeout(1.5),
      new ShootBallSpecific(m_shooter, m_indexer, 5800, 1600)
    );

    this.chooseAuton.addOption("SixBall Auton", sixBallAuton);
    this.chooseAuton.addOption("ThreeBall Auton", threeBallAuton);
  }

  private void configureAutonChooser() {

    SmartDashboard.putData("Auto Chooser", this.chooseAuton);

    // String[] paths = { "leftTurn", "line", "oneMeter1", "rightTurn", "slalom", "slalom1", "test", "twoMeter" };

    // for (String s : paths) {
    //   this.chooseAuton.addOption(s, this.loadTrajectoryCommand(s));
    // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(chooseAuton.getSelected() != null) {
      return chooseAuton.getSelected();
    } else {
      return threeBallAuton;
    }
  }

  public Command getAutonomousShootCommand() {
    SequentialCommandGroup driveAndShoot = new SequentialCommandGroup(getAutonomousCommand().withTimeout(8),
        shootBallInterpolated.withTimeout(5));
    return driveAndShoot;
  }

  public Command getTeleopInitCommand() {
    return resetHood;
  }

  public void onTeleopDisable() {
    m_driveTrain.setNeutralMode(NeutralMode.Coast);
    m_limelight.turnOnLED();
  }

  public void onTeleopEnable() {
    m_driveTrain.setNeutralMode(NeutralMode.Brake);
  }

  public RamseteDriveSubsystem getDriveSubsystem() {
    return m_driveTrain;
  }

  protected Command loadTrajectoryCommand(String trajectoryName) {
    Trajectory t = loadTrajectory(trajectoryName);
    return new DriveTrajectory(m_driveTrain, t).andThen(() -> m_driveTrain.tankDriveVolts(0.0, 0.0));
  }

  public static Trajectory loadTrajectory(String trajectoryName) {
    try {
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath()
          .resolve(Paths.get("paths", "output", trajectoryName + ".wpilib.json")));
      return trajectory;
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + trajectoryName, false);
      return null;
    }
  }
}
