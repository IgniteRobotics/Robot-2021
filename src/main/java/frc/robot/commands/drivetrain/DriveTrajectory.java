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

    public DriveTrajectory(RamseteDriveSubsystem ramsetedriveTrain, Trajectory trajectory){
        // public RamseteCommand
        //     Trajectory trajectory,
        //     Supplier<Pose2d> pose,
        //     RamseteController controller,
        //     SimpleMotorFeedforward feedforward,
        //     DifferentialDriveKinematics kinematics,
        //     Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
        //     PIDController leftController,
        //     PIDController rightController,
        //     BiConsumer<Double, Double> outputVolts,
        //     Subsystem... requirements)
        super(trajectory, 
            ramsetedriveTrain::getCurrentPose, 
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts, 
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter), 
            new DifferentialDriveKinematics (Constants.kTrackwidthMeters), 
            ramsetedriveTrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            ramsetedriveTrain::tankDriveVolts, 
            ramsetedriveTrain);        
        this.ramseteDriveTrain = ramsetedriveTrain;
        this.trajectory = trajectory;
    }


    @Override
    public void initialize() {
        super.initialize();
        ramseteDriveTrain.resetOdometry(trajectory.getInitialPose());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}