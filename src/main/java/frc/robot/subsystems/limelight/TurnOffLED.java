// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnOffLED extends CommandBase {
  private LimelightSubsystem limelight;

  /**
   * Turn off LED if target detected
   * @param limesub
   */
  public TurnOffLED(LimelightSubsystem limesub) {
    this.limelight = limesub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn on limelight in the beginning
    limelight.setLED(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // turn off if target detected
    if (limelight.getTv() == 1) {
      limelight.setLED(1);
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
