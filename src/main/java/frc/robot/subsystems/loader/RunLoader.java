// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunLoader extends CommandBase {
  private LoaderSubsystem loaderSubsystem;
  private double voltage;

  public RunLoader(LoaderSubsystem loaderSubsystem, double voltage) {
    this.loaderSubsystem = loaderSubsystem;
    this.voltage = voltage;
    addRequirements(loaderSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    loaderSubsystem.setVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    loaderSubsystem.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
