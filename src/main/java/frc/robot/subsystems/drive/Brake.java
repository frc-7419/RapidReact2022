// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Brake extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  /** Creates a new Brake. */
  public Brake(DriveBaseSubsystem driveBaseSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBaseSubsystem = driveBaseSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBaseSubsystem.brake();
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
