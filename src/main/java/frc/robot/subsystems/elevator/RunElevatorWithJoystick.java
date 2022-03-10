// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunElevatorWithJoystick extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private XboxController joystick;

  public RunElevatorWithJoystick(ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.coast();
  }

  @Override
  public void execute() {

    SmartDashboard.putNumber("joystick out", joystick.getRightY());

    if (joystick.getRightY() != 0) {
      elevatorSubsystem.coast();
      elevatorSubsystem.setPower(-joystick.getRightY() * 0.35);
    } else {
      elevatorSubsystem.setPower(0);
      elevatorSubsystem.brake();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
