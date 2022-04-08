// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunElevatorWithJoystick extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private XboxController joystick1;
  private XboxController joystick2;

  public RunElevatorWithJoystick(ElevatorSubsystem elevatorSubsystem, XboxController joystick1, XboxController joystick2) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick1 = joystick1;
    this.joystick2 = joystick2;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.coast();
  }

  @Override
  public void execute() {

    // SmartDashboard.putNumber("joystick out", joystick.getLeftY());

    if (joystick1.getLeftY() != 0) {
      elevatorSubsystem.coast();
      elevatorSubsystem.setPower(-joystick1.getLeftY() * 0.32);
    } else if (joystick2.getLeftY() != 0) {
      elevatorSubsystem.coast();
      elevatorSubsystem.setPower(-joystick2.getLeftY() * 0.32);
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
