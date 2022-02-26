// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunTurretWithJoystick extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private XboxController joystick;
  private double kSpeed;

  public RunTurretWithJoystick(TurretSubsystem turretSubsystem, XboxController joystick, double kSpeed) {
    this.turretSubsystem = turretSubsystem;
    this.joystick = joystick;
    this.kSpeed = kSpeed;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turretSubsystem.setPower(kSpeed*joystick.getLeftX());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
