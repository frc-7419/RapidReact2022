// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DeployIntake extends CommandBase {
  private IntakeSolenoidSubsystem intakeSolenoidSubsystem;
  private XboxController joystick;

  public DeployIntake(IntakeSolenoidSubsystem intakeSolenoidSubsystem, XboxController joystick) {
    this.intakeSolenoidSubsystem = intakeSolenoidSubsystem;
    this.joystick = joystick;
    addRequirements(intakeSolenoidSubsystem);
  }

  @Override
  public void initialize() {
    intakeSolenoidSubsystem.retractSolenoid();
  }

  @Override
  public void execute() {
    if (joystick.getXButtonPressed()) {
      intakeSolenoidSubsystem.toggleSolenoid();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSolenoidSubsystem.retractSolenoid();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
