// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AlternateRunElevatorWithJoystick extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private XboxController joystick;

  public AlternateRunElevatorWithJoystick(ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
    addRequirements(elevatorSubsystem);
  }

  public static enum Direction {
    UP(0), RIGHT(90), DOWN(180), LEFT(270);
    int direction;

    private Direction(int direction) {
        this.direction = direction;
    }
  } 

  @Override
  public void initialize() {
    elevatorSubsystem.coast();
  }

  @Override
  public void execute() {
    int dPadValue = joystick.getPOV();
    if (dPadValue == Direction.UP.direction) {
      elevatorSubsystem.coast();
      elevatorSubsystem.setPower(0.32);
    }
    else if (dPadValue == Direction.DOWN.direction) {
      elevatorSubsystem.coast();
      elevatorSubsystem.setPower(-0.32);
    }
    else {
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