// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transferWheel;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunTransferWheel extends CommandBase {
  private XboxController joystick;
  private TransferWheelSubsystem transferWheelSubsystem;
  private double power;

  public RunTransferWheel(XboxController joystick, TransferWheelSubsystem transferWheelSubsystem, double power) {
    this.joystick = joystick;
    this.transferWheelSubsystem = transferWheelSubsystem;
    this.power = power;
    addRequirements(transferWheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transferWheelSubsystem.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transferWheelSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
