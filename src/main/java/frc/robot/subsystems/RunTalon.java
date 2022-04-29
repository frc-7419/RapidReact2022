// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunTalon extends CommandBase {
  private TalonSubsystem talonSubsystem;
  private XboxController joystick;

  public RunTalon(TalonSubsystem talonSubsystem, XboxController joystick) {
    this.talonSubsystem = talonSubsystem;
    this.joystick = joystick;
    addRequirements(talonSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    talonSubsystem.setPower(joystick.getLeftY()*0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    talonSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
