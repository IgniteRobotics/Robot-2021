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
import frc.robot.commands.ShootBall;
import frc.robot.commands.TestExtendShooterHood;
import frc.robot.commands.TestRetractHood;
import frc.robot.commands.runIndexer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Intake.RunIntake;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.drivetrain.ToggleSlowMode;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.drivetrain.TargetPositioning;

import frc.robot.commands.drivetrain.DriveTrajectory;
import frc.robot.commands.drivetrain.RamseteArcadeDrive;
import frc.robot.subsystems.RamseteDriveSubsystem;
/**
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private RamseteDriveSubsystem m_driveTrain = new RamseteDriveSubsystem();
  private Intake m_intake = new Intake();
  private Shooter m_shooter = new Shooter();
  private Indexer m_indexer = new Indexer();
  private Limelight m_limelight = new Limelight();
  
  private Joystick m_driveController = new Joystick(Constants.kDriveControllerPort);
  private Joystick m_manipController = new Joystick(Constants.kManipControllerPort);
  
  
  
  
  //private ArcadeDrive teleDriveCommand = new ArcadeDrive(m_driveController, m_driveTrain);
  private RamseteArcadeDrive teleDriveCommand = new RamseteArcadeDrive(m_driveController, m_driveTrain);
  //rivate AutoForward m_auto = new AutoForward(m_driveTrain, 1000);
  
  private ShootBall shootCommand = new ShootBall(m_shooter, m_indexer, m_limelight);
  
  SendableChooser<Command> chooseAuton = new SendableChooser<>();

  private SequentialCommandGroup shootSequence = new SequentialCommandGroup(new TurnToAngle(m_driveTrain, m_limelight), shootCommand);
  
  
  
  
  
  
  
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
    this.configureButtonBindings();
    this.configureSubsystemCommands();
    this.configureAutonChooser();
  }
  
  /**
  * Use this method to define your button->command mappings.  Buttons can be created by
  * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
    private void configureButtonBindings() {
      //new JoystickButton(m_driveController, Constants.BUTTON_X).whileHeld(new ExampleCommand(m_driveTrain);
      // new JoystickButton(m_driveController, Constants.BUTTON_LEFT_BUMPER).whileHeld(new ToggleSlowMode(m_driveTrain));
      // new JoystickButton(m_driveController, Constants.BUTTON_A).whileHeld(new TargetPositioning(m_driveTrain, 138));
      // new JoystickButton(m_driveController, Constants.BUTTON_B).whileHeld(new TargetPositioning(m_driveTrain, 222));
      
      //should this be manipulator or driver controller?
      //if you want to increase the speed of the intake increase this number up to max of 1
      new JoystickButton(m_manipController, Constants.BUTTON_LEFT_BUMPER).whileHeld(new RunIntake(0.6, m_intake));
      new JoystickButton(m_manipController, Constants.BUTTON_X).whileHeld(new runIndexer(0.20, m_indexer)); //check if this works
      
      //adding driver control for intake and shooter
      new JoystickButton(m_driveController, Constants.BUTTON_B).whileHeld(new RunIntake(0.6, m_intake));
      
      new JoystickButton(m_manipController, Constants.BUTTON_A).whileHeld(shootSequence);
      new JoystickButton(m_driveController, Constants.BUTTON_X).whenPressed(new TestExtendShooterHood(m_shooter));
      new JoystickButton(m_driveController, Constants.BUTTON_Y).whenPressed(new TestRetractHood(m_shooter));
      
      
    }
    
    private void configureSubsystemCommands() {
      m_driveTrain.setDefaultCommand(teleDriveCommand);
    }
    
    private void configureAutonChooser() {
      
      SmartDashboard.putData("Auto Chooser", this.chooseAuton);
      
      String[] paths = {
        "28-BarrelRacing6",
        "28-Bounce-A3",
        "28-Bounce-A6",
        "28-Bounce-A9",
        "28-Bounce-finish",
        "28-GS-A-Blue",
        "28-GS-A-Red",
        "28-GS-B-Blue",
        "28-GS-B-Red",
        "28-Slalom",
        "Sam's Barrol",
        "Sam's Bounce",
        "Sam's Slalom",
        "barrel1",
        "barrelracing",
        "bounceRace1",
        "halfSlalom",
        "leftTurn",
        "line",
        "oneMeter1",
        "rightTurn",
        "slalom",
        "slalom1",
        "test",
        "twoMeter"
      };
      
      for (String s: paths){
        this.chooseAuton.addOption(s, this.loadTrajectory(s));
      }
      
    }
    
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
      System.out.println("Auto: " + chooseAuton.getSelected().getName());
      return chooseAuton.getSelected();
    }
    
    protected Command loadTrajectory(String trajectoryName) {
      try{
        Trajectory trajectory =  TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", "output", trajectoryName + ".wpilib.json")));
        return new DriveTrajectory(m_driveTrain, trajectory).andThen(() -> m_driveTrain.tankDriveVolts(0.0, 0.0));
        
      } catch(IOException e) {
        DriverStation.reportError("Failed to load auto trajectory: " + trajectoryName, false);
        return null;
      }
    }
  }
  