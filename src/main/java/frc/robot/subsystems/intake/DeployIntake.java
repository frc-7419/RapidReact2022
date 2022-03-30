// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeployIntake extends CommandBase {
  private IntakeSolenoidSubsystem intakeSolenoidSubsystem;

  public DeployIntake(IntakeSolenoidSubsystem intakeSolenoidSubsystem) {
    this.intakeSolenoidSubsystem = intakeSolenoidSubsystem;
    addRequirements(intakeSolenoidSubsystem);
  }

  @Override
  public void initialize() {
    intakeSolenoidSubsystem.actuateSolenoid();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
