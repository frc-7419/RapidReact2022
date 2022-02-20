// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pneumatics.SolenoidSubsystem;

public class DeployIntakeWithPneumatics extends CommandBase {
  private SolenoidSubsystem solenoidSubsystem;
  private XboxController joystick;
  /** Creates a new DeployIntakeWithPneumatics. */
  public DeployIntakeWithPneumatics(SolenoidSubsystem solenoidSubsystem, XboxController joystick) {
    this.solenoidSubsystem = solenoidSubsystem;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(solenoidSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getXButtonPressed()) {
      solenoidSubsystem.toggleSolenoid();
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
