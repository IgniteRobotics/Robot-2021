/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RamseteDriveSubsystem;
import frc.robot.constants.MotorConstants;

public class RamseteDrive extends CommandBase { // TODO Figure out how to make a button trigger slow mode

    private final RamseteDriveSubsystem m_driveTrain;
    private final Joystick driverJoystick;

    private boolean isSlowMode = false; // Figure out a button for this.
    private boolean isReversed = false;

    /**
     * Creates a new ArcadeDrive.
     */
    public RamseteDrive(Joystick driveController, RamseteDriveSubsystem driveTrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driverJoystick = driveController;
        this.m_driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    //Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        ShuffleboardTab tab = Shuffleboard.getTab("DriveTrain");
        NetworkTableEntry driveMode = tab.add("Choose Control mode", "Arcade Drive").getEntry();

        if (driveMode.getString("Arcade Drive") == "Arcade Drive") {
            m_driveTrain.arcadeDrive(getSpeed(), getRotation(), true);
            outputTelemetry();
        } else {
            m_driveTrain.driveCurvature(getSpeed(), getRotation(), true);
            outputTelemetry();
        }
    }

    private double getSpeed() {
        double speed = -driverJoystick.getRawAxis(ControllerConstants.AXIS_LEFT_STICK_Y);
        // if(m_driveTrain.isSlowMode) {
        //   speed *= Constants.SLOW_MODE_SPEED_MODIFIER;
        // }
        return speed;
    }

    private double getRotation() {
        double rotation = (driverJoystick.getRawAxis(ControllerConstants.AXIS_RIGHT_STICK_X));
        // if(m_driveTrain.isSlowMode) {
        //   rotation *= Constants.SLOW_MODE_SPEED_MODIFIER;
        // }
        return rotation;
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("RamseteAD/Speed", this.getSpeed());
        SmartDashboard.putNumber("RamseteAD/Rotation", this.getRotation());
    }
}
