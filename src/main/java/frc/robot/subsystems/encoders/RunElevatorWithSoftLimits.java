// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RunElevatorWithSoftLimits extends CommandBase {
  /** Creates a new RunElevatorWithSoftLimits. */
  private ElevatorSubsystem elevatorSubsystem;
  private XboxController joystick;
  private double forwardSoftLimit;
  private double reverseSoftLimit;

  public RunElevatorWithSoftLimits(ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
    
    //Change value for soft limits based on the limits in ticks of elevator
    //Should be elevatorSubsystem.inchesToTicks(inches, diameter) with inches and diameter changed
    forwardSoftLimit = elevatorSubsystem.inchesToTicks(20, 0.525, 21);
    reverseSoftLimit = elevatorSubsystem.inchesToTicks(-20, 0.525, 21);
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.getElevatorLeft().configForwardSoftLimitThreshold(forwardSoftLimit);
    elevatorSubsystem.getElevatorLeft().configReverseSoftLimitThreshold(reverseSoftLimit);
    elevatorSubsystem.getElevatorLeft().configForwardSoftLimitEnable(true, 0);
    elevatorSubsystem.getElevatorLeft().configReverseSoftLimitEnable(true, 0);

    elevatorSubsystem.getElevatorRight().configForwardSoftLimitThreshold(forwardSoftLimit);
    elevatorSubsystem.getElevatorRight().configReverseSoftLimitThreshold(reverseSoftLimit);
    elevatorSubsystem.getElevatorRight().configForwardSoftLimitEnable(true, 0);
    elevatorSubsystem.getElevatorRight().configReverseSoftLimitEnable(true, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Left Elevator Position", elevatorSubsystem.getElevatorLeft().getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Elevator Position", elevatorSubsystem.getElevatorRight().getSelectedSensorPosition());
    SmartDashboard.putNumber("Joystick value", joystick.getRightY());

    if (joystick.getRightY() != 0){
      elevatorSubsystem.setPower(0.3 * joystick.getRightY());
    }
    else{
      elevatorSubsystem.setPower(0);
      elevatorSubsystem.brake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
