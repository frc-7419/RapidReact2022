// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class TestCAN extends CommandBase {
  /** Creates a new TestCAN. */
  private DriveBaseSubsystem driveBaseSubsystem;
  public TestCAN(DriveBaseSubsystem driveBaseSubsystem) {
    this.driveBaseSubsystem = driveBaseSubsystem;
    addRequirements(driveBaseSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called wh en the command is initially scheduled.
  @Override
  public void initialize() {
    driveBaseSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBaseSubsystem.getLeftMast().set(ControlMode.PercentOutput, 0.1);
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
