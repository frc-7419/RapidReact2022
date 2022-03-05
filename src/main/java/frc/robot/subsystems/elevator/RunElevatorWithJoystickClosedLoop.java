// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;


public class RunElevatorWithJoystickClosedLoop extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private XboxController joystick;

  private double initialPosition;
  private double currentPosition;

  public RunElevatorWithJoystickClosedLoop(ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.coast();
    currentPosition = elevatorSubsystem.getElevatorPosition();
  }

  @Override
  public void execute() {
    if (joystick.getRightY() != 0) {
      elevatorSubsystem.coast();
      elevatorSubsystem.setPower(joystick.getRightY() * 0.35);
      currentPosition = elevatorSubsystem.getElevatorPosition();
    } else {
      elevatorSubsystem.setPIDFConstants(PIDConstants.ElevatorKp, 0, 0, 0);
      elevatorSubsystem.getElevatorLeft().set(ControlMode.MotionMagic, currentPosition, DemandType.ArbitraryFeedForward, PIDConstants.ElevatorKf);
    } 
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
