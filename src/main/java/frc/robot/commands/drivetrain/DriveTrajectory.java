package frc.robot.commands.drivetrain;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.RamseteDriveSubsystem;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory; 
import edu.wpi.first.wpilibj.controller.RamseteController;
import frc.robot.Constants;
public class DriveTrajectory extends RamseteCommand {

    private RamseteDriveSubsystem ramseteDriveTrain;
    private Trajectory trajectory;

    public DriveTrajectory(RamseteDriveSubsystem ramsetedriveTrain, Trajectory trajectory){ //change this to follow trajceoty later
        super(
            trajectory, 
            ramsetedriveTrain::getCurrentPose, 
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
            new DifferentialDriveKinematics (Constants.kTrackwidthMeters), 
            ramsetedriveTrain::tankDriveVelocity, 
            ramsetedriveTrain
        );       //tune ramsete controller 
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