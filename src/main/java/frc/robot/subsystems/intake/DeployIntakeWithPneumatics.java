// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limitswitch.LimitSwitchSubsystem;
import frc.robot.subsystems.pneumatics.SolenoidSubsystem;

public class DeployIntakeWithPneumatics extends CommandBase {
  private SolenoidSubsystem solenoidSubsystem;
  private LimitSwitchSubsystem limitSwitchSubsystem;
  /** Creates a new DeployIntakeWithPneumatics. */
  public DeployIntakeWithPneumatics(SolenoidSubsystem solenoidSubsystem, LimitSwitchSubsystem limitSwitchSubsystem) {
    this.solenoidSubsystem = solenoidSubsystem;
    this.limitSwitchSubsystem = limitSwitchSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(solenoidSubsystem, limitSwitchSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limitSwitchSubsystem.get()) {
      solenoidSubsystem.retractSolenoid();
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
