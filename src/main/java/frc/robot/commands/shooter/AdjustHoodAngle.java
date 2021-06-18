// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class AdjustHoodAngle extends CommandBase {
  private Shooter shooter;

  private ShuffleboardTab shuffleTab;
  private NetworkTableEntry hoodAngleEntry;

  private double targetAngle;

  /** Creates a new AdjustHoodAngle. */
  public AdjustHoodAngle(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.

    shuffleTab = Shuffleboard.getTab("Shooter");
    hoodAngleEntry = shuffleTab.add("Shooter Angle", shooter.getHoodAngle()).getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = hoodAngleEntry.getDouble(shooter.getHoodAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getHoodAngle() != targetAngle) {
      shooter.changeHoodAngle(targetAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
