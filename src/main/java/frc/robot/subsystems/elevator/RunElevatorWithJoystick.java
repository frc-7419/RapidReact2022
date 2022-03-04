// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunElevatorWithJoystick extends CommandBase {
  /** Creates a new RunElevatorWithLimitSwitch. */
  private ElevatorSubsystem elevatorSubsystem;
  private XboxController joystick;

  public RunElevatorWithJoystick(ElevatorSubsystem elevatorSubsystem, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystick = joystick;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
