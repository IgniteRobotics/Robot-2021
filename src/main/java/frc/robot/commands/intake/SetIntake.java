// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetIntake extends CommandBase {
  private Intake intake;

  private boolean extend;

  public SetIntake(Intake intake, boolean extend) {
    this.intake = intake;
    this.extend = extend;

    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(extend) {
      intake.extendIntake();
    } else {
      intake.retractIntake();
    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return intake.isExtended() == extend;
  }
}
