// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunLoader extends CommandBase {
  private LoaderSubsystem loaderSubsystem;
  private XboxController joystick;
  private double power;

  public RunLoader(LoaderSubsystem loaderSubsystem, XboxController joystick, double power) {
    this.loaderSubsystem = loaderSubsystem;
    this.joystick = joystick;
    addRequirements(loaderSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    loaderSubsystem.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
