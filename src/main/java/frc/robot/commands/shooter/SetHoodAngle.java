// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.util.ShuffleBoardShootBall;

public class SetHoodAngle extends CommandBase {
  /** Creates a new SetHoodAngle. */
  //Command that sets the angle of the hood. Angle (in ticks) is taken from shuffleboard. Meant to be run in test mode
  //Ideally, this would be angle, but ticks will work fine too
  private NetworkTableEntry targetHoodTicksEntry;
  private Shooter shooter;
  private double targetHoodTicks;

  public SetHoodAngle(Shooter shooter) {
  
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetHoodTicks = ShuffleBoardShootBall.getInstance().getTargetHoodTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   shooter.changeHoodTicks(targetHoodTicks);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
