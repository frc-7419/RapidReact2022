// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnOffLED extends CommandBase {
  private LimelightSubsystem limelightSubsystem;

  /**
   * Turn off LED if target detected
   * @param limelightSubsystem
   */
  public TurnOffLED(LimelightSubsystem limelightSubsystem) {
    this.limelightSubsystem = limelightSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn on limelight in the beginning
    limelightSubsystem.setLED(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // turn off if target detected
    if (limelightSubsystem.getTv() == 1) {
      limelightSubsystem.setLED(1);
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
