// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rev2mDistanceSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunRev2mDistanceSensor extends CommandBase {
  /** Creates a new RunRev2mDistanceSensor. */
  private Rev2mDistanceSensorSubsystem rev2mDistanceSensorSubsystem;

  public RunRev2mDistanceSensor(Rev2mDistanceSensorSubsystem rev2mDistanceSensorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rev2mDistanceSensorSubsystem = rev2mDistanceSensorSubsystem;
    addRequirements(rev2mDistanceSensorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rev2mDistanceSensorSubsystem.setAutomaticMode(true);
    if(rev2mDistanceSensorSubsystem.getDistanceSensor().isRangeValid()) {
      SmartDashboard.putNumber("Range Onboard", rev2mDistanceSensorSubsystem.getRange());
      SmartDashboard.putNumber("Timestamp Onboard", rev2mDistanceSensorSubsystem.getTimeStamp());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rev2mDistanceSensorSubsystem.setAutomaticMode(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
