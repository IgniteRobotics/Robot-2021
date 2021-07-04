package frc.robot.commands.climber;

import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControllerConstants;

public class ClimbJoystick extends CommandBase {
  private Climber climb;
  private Joystick climbJoystick;

  private final double effortLimit = 0.25;

  /** Creates a new ClimbUp. */
  public ClimbJoystick(Climber climb, Joystick climbJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
     this.climb = climb;
     this.climbJoystick = climbJoystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double effort = climbJoystick.getRawAxis(ControllerConstants.AXIS_LEFT_STICK_Y);
      double convertedEffort = (effort / 1) * effortLimit;
      climb.go(convertedEffort);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
