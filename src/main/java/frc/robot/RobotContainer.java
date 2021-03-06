/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.runIndexer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Intake.RunIntake;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.drivetrain.ToggleSlowMode;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.drivetrain.TargetPositioning;
import frc.robot.commands.drivetrain.Slalom;
import frc.robot.subsystems.RamseteDriveSubsystem;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private DriveTrain m_driveTrain = new DriveTrain(Constants.kLeftMasterPort, Constants.kLeftFollowerPort, Constants.kLeftFollowerPort2, 
  Constants.kRightMasterPort, Constants.kRightFollowerPort, Constants.kRightFollowerPort2);
private RamseteDriveSubsystem ramsetedriveTrain = new RamseteDriveSubsystem();
private Intake m_intake = new Intake();
private Shooter m_shooter = new Shooter();
private Indexer m_indexer = new Indexer();

private Joystick m_driveController = new Joystick(Constants.kDriveControllerPort);
private Joystick m_manipController = new Joystick(Constants.kManipControllerPort);


private ArcadeDrive teleDriveCommand = new ArcadeDrive(m_driveController, m_driveTrain);
//rivate AutoForward m_auto = new AutoForward(m_driveTrain, 1000);

//SendableChooser chooseAuton = new SendableChooser<>();
//m_chooser.addOption("Complex Auto", m_complexAuto); //Change these to our auton commands



 


  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  //Let's store our auton commands here and hope that this is a good place to store them

  //put name of challenge and then whatever it's used for 
//private final command autoNavPath1;
//private final command autoNavPath2;
//private final command autoNavPath3;


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(m_driveController, Constants.BUTTON_X).whileHeld(new ExampleCommand(m_driveTrain);
    new JoystickButton(m_driveController, Constants.BUTTON_LEFT_BUMPER).whileHeld(new ToggleSlowMode(m_driveTrain)); //TODO fix this
    new JoystickButton(m_driveController, Constants.BUTTON_A).whileHeld(new TargetPositioning(m_driveTrain, 138));
    new JoystickButton(m_driveController, Constants.BUTTON_B).whileHeld(new TargetPositioning(m_driveTrain, 222));

    //should this be manipulator or driver controller?
    //if you want to increase the speed of the intake increase this number up to max of 1
    new JoystickButton(m_manipController, Constants.BUTTON_LEFT_BUMPER).whileHeld(new RunIntake(0.6, m_intake));
    new JoystickButton(m_manipController, Constants.BUTTON_X).whileHeld(new runIndexer(0.20, m_indexer)); //check if this works
  }

  private void configureSubsystemCommands() {
    m_driveTrain.setDefaultCommand(teleDriveCommand);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Each trejectory will have its own command and we will choose which one to run in auton thorugh smartdashboard
    //there are around 3 different paths for autonomous

    Trajectory trajectory = loadTrajectory("slalom");
    if(trajectory == null) {
      return null;
    }


    //remember to change this to follow trajecotry
    return new Slalom(ramsetedriveTrain, trajectory);
  }

  protected static Trajectory loadTrajectory(String trajectoryName) {
    try{
    return TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", "output", trajectoryName + ".wpilib.json")));
    } catch(IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + trajectoryName, false);
      return null;
    }
  }
}
