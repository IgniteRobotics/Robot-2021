// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.util.StateMachine;
import frc.robot.util.shooter.InterpolationCalculator;
import frc.robot.util.shooter.ShooterParameter;
import frc.robot.util.ShuffleBoardShootBall;

import java.util.Map;
//Picks the right RPM and angle to shoot the ball at depending on limelight-reported distance
public class ShootInterpolatedBall extends CommandBase {
    private Shooter shooter;
    private Indexer indexer;

    private ShuffleBoardShootBall shuffle = ShuffleBoardShootBall.getInstance();

    private Limelight limelight;

    private static final int RANGE = 150;
    private static final double ANGLE_RANGE = 5;

    private double targetVelocity = Constants.HOOD_DEFAULT_RPM;
    private double intakeEffort = 0.4;
    private double kickupEffort = 0.3;
    private double distanceSetpoint = Constants.HOOD_SET_POINT_DISTANCE;

    private InterpolationCalculator calculator = new InterpolationCalculator();

    public ShootInterpolatedBall(Shooter shooter, Indexer indexer, Limelight limelight) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.limelight = limelight;
        addRequirements(shooter, indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeEffort = shuffle.getIntakeEffort();
        kickupEffort = shuffle.getKickupEffort();
        distanceSetpoint = shuffle.getDistanceSetPoint();
        double currentDistance = limelight.getDistancefromgoal();

        ShooterParameter calculatedParameters = calculator.calculateParameter(currentDistance);
        targetVelocity = calculatedParameters.rpm;

        setShooterRPM((int) targetVelocity); // use rpm from interpolation
        shooter.changeHoodTicks(calculatedParameters.angle);

        double shooterRPM = shooter.getShooterRPM();

        // if((targetVelocity - RANGE <= shooterRPM && targetVelocity + RANGE >= shooterRPM) &&
        //     (shooterAngle - ANGLE_RANGE < computedAngle && shooterAngle + ANGLE_RANGE > computedAngle)) {
        if ((targetVelocity - RANGE <= shooterRPM && targetVelocity + RANGE >= shooterRPM)
            //  && shooter.isHoodReady()
        ) {
            shooter.runKickup(kickupEffort);
            indexer.runIndexer(intakeEffort);
        } else {
            shooter.stopKickup();
            indexer.runIndexer(0);
        }
    }

    private void setShooterVelocity(double velocity) {
        if (velocity >= 0) {
            shooter.setVelocity(velocity);
        }
    }

    private void setShooterRPM(int rpm) {
        if (rpm >= 0) {
            shooter.setRPM(rpm);
        } else {
            shooter.setRPM(0);
        }
    }


    private void stop() {
        shooter.setpower(0.0);
        shooter.stopKickup();
        indexer.stop();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
