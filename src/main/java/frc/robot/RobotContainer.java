/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.drivetrain.ToggleSlowMode;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.drivetrain.TargetPositioning;


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
// private RamseteDriveSubsystem m_driveTrain = new RamseteDriveSubsystem();
private Intake m_intake = new Intake();
private Shooter m_shooter = new Shooter();

private Joystick m_driveController = new Joystick(Constants.kDriveControllerPort);
private Joystick m_manipController = new Joystick(Constants.kManipControllerPort);
private ArcadeDrive teleDriveCommand = new ArcadeDrive(m_driveController, m_driveTrain);
//rivate AutoForward m_auto = new AutoForward(m_driveTrain, 1000);










  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);



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
    new JoystickButton(m_driveController, Constants.BUTTON_LEFT_BUMPER).whileHeld(new ToggleSlowMode(m_driveTrain));
    new JoystickButton(m_driveController, Constants.BUTTON_A).whileHeld(new TargetPositioning(m_driveTrain, 138));
    new JoystickButton(m_driveController, Constants.BUTTON_B).whileHeld(new TargetPositioning(m_driveTrain, 222));

    
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
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
