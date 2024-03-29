package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.RamseteDriveSubsystem;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.constants.Constants;
public class DriveTrajectory extends RamseteCommand {

    private RamseteDriveSubsystem ramseteDriveTrain;
    private Trajectory trajectory;

    // public DriveTrajectory(RamseteDriveSubsystem ramsetedriveTrain, Trajectory trajectory){ //change this to follow trajceoty later
    //     super(trajectory, ramsetedriveTrain::getCurrentPose, new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), new DifferentialDriveKinematics (Constants.kTrackwidthMeters), ramsetedriveTrain::tankDriveVelocity, ramsetedriveTrain);       //tune ramsete controller 
    //     this.ramseteDriveTrain = ramsetedriveTrain;
    //     this.trajectory = trajectory;
    // }

    public DriveTrajectory(RamseteDriveSubsystem drive, Trajectory trajectory){
        super(
            trajectory,
            drive::getCurrentPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            drive::tankDriveVolts,
            drive
          );  

        this.ramseteDriveTrain = drive;
        this.trajectory = trajectory;
    }


    @Override
    public void initialize() {
        super.initialize();
        ramseteDriveTrain.resetEncoders();
        ramseteDriveTrain.resetOdometry(trajectory.getInitialPose());
    }
}