// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.revMagneticLimitSwitch.RevMagneticLimitSwitchSubsystem;

public class RunTurretWithREVMagneticSwitch extends CommandBase {
  private TurretSubsystem turretSubsystem;
  private XboxController joystick;
  private RevMagneticLimitSwitchSubsystem leftSwitch;
  private RevMagneticLimitSwitchSubsystem rightSwitch;
  private double kSpeed;

  public RunTurretWithREVMagneticSwitch(TurretSubsystem turretSubsystem, XboxController joystick, RevMagneticLimitSwitchSubsystem leftSwitch, RevMagneticLimitSwitchSubsystem rightSwitch, double kSpeed) {
    this.turretSubsystem = turretSubsystem;
    this.joystick = joystick;
    this.leftSwitch = leftSwitch;
    this.rightSwitch = rightSwitch;
    this.kSpeed = kSpeed;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!(this.leftSwitch.get() && this.rightSwitch.get())) {
      turretSubsystem.setPower(this.kSpeed*this.joystick.getLeftY());
    }
    else {
      turretSubsystem.setPower(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
