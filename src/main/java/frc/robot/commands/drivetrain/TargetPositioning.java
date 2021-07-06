/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RamseteDriveSubsystem;

import java.util.Map;

import javax.sound.sampled.LineListener;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.StateMachine;
import frc.robot.util.VisionUtils;


public class TargetPositioning extends CommandBase {
    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static double KpTurn = 0.015;// or 0.035
    //private static double minCommand = 0.08;
    private static double minCommand = 0.08;
    private static double minInfPoint = 4; // or 4
    // the range you want.
    //allowed margin of errorit
    private double marginOfErrorTurn = 2.0;

    boolean targetFound = false;

    private StateMachine state;

    private final RamseteDriveSubsystem m_driveTrain;
    private final Joystick driverJoystick;

    private Limelight limelight;

    /**
     * Creates a new TargetRange.
     */
    public TargetPositioning(RamseteDriveSubsystem driveTrain, Joystick driveController, Limelight limelight) {
        addRequirements(driveTrain);
        this.m_driveTrain = driveTrain;
        this.driverJoystick = driveController;
        this.limelight = limelight;
        addRequirements(limelight);
        // Use addRequirements() here to declare subsystem dependencies.

        state = StateMachine.getIntance();
    }

    // Called wen the command is initially scheduled.
    @Override
    public void initialize() {
        targetFound = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double tv = limelight.getTargetViewable();
        SmartDashboard.putNumber("Limelight tv", tv);

        if (Math.abs(tv) > 0) {

            targetFound = true;

            double tx = limelight.getHorizontalOffset();
            double ty = limelight.getVerticalOffset();

            double dist = VisionUtils.getDistanceToTarget(ty);

            SmartDashboard.putNumber("Limelight tx", tx);
            SmartDashboard.putNumber("Limelight ty", ty);
            SmartDashboard.putNumber("Limelight dist", dist);

            state.setShooterDistance(dist);

            double headingError = Math.abs(tx);
            double steeringAdjust = 0.0;
            steeringAdjust = -(KpTurn * headingError + minCommand) * Math.signum(tx);


            //m_driveTrain.arcadeDrive(-drivingAdjust,steeringAdjust,Constants.kDriveDeadband);
            //flip it since the shooter is on the back.
            SmartDashboard.putNumber("TargetPositioning/SteeringAdjust", steeringAdjust);
            m_driveTrain.arcadeDrive(0, -steeringAdjust, false);
        } else {
            targetFound = false;
            m_driveTrain.arcadeDrive(getSpeed(), getRotation(), true);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
     //   m_driveTrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    
    private double getRotation() {
        double rotation = (driverJoystick.getRawAxis(ControllerConstants.AXIS_RIGHT_STICK_X));
        // if(m_driveTrain.isSlowMode) {
        //   rotation *= Constants.SLOW_MODE_SPEED_MODIFIER;
        // }
        return rotation;
    }

    
    private double getSpeed() {
        double speed = -driverJoystick.getRawAxis(ControllerConstants.AXIS_LEFT_STICK_Y);
        // if(m_driveTrain.isSlowMode) {
        //   speed *= Constants.SLOW_MODE_SPEED_MODIFIER;
        // }
        return speed;
    }


}