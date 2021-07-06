// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RetractClimbMax extends CommandBase {
  private Climber climber;

  public RetractClimbMax(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setNoLimits(true);
    climber.resetCurrentLimits();
  }

  @Override
  public void execute() {
    climber.reduceMaxSafe();
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
    climber.setNoLimits(false);
  }

  @Override
  public boolean isFinished() {
    return climber.bothCurrentStopped();
  }
}
