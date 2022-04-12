// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunLoaderWithJoystick extends CommandBase {
  private LoaderSubsystem loaderSubsystem;
  private XboxController joystick;
  private double voltage;

  public RunLoaderWithJoystick(LoaderSubsystem loaderSubsystem, XboxController joystick, double voltage) {
    this.loaderSubsystem = loaderSubsystem;
    this.joystick = joystick;
    this.voltage = voltage;
    addRequirements(loaderSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (joystick.getLeftBumper()) {
      loaderSubsystem.setVoltage(voltage);
      loaderSubsystem.coast();
    }
    else if (joystick.getRightBumper()) {
      loaderSubsystem.setVoltage(voltage);
      loaderSubsystem.coast();
    }
    else {
      loaderSubsystem.setVoltage(0);
      // loaderSubsystem.brake();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
