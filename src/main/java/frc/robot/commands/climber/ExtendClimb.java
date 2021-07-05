package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Extends the climb to the maximum height.
 */
public class ExtendClimb extends CommandBase {
  private static final double effort = 0.5;
  private static final double limitThreshold = 100;

  private Climber climber;

  public ExtendClimb(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    int currentPos = climber.getEncoderPos();

    if(!(currentPos <= Climber.CLIMBER_FORWARD_LIMIT + limitThreshold && currentPos >= Climber.CLIMBER_FORWARD_LIMIT - limitThreshold)) {
      climber.go(effort);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    int currentPos = climber.getEncoderPos();
    return (currentPos <= Climber.CLIMBER_FORWARD_LIMIT + limitThreshold && currentPos >= Climber.CLIMBER_FORWARD_LIMIT - limitThreshold);
  }
}
