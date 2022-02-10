// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.limitswitch.LimitSwitchSubsystem;

public class RetractIntakeWithElevator extends CommandBase {
  /** Creates a new RetractIntakeWithElevator. */
  private ElevatorSubsystem elevatorSubsystem;
  private LimitSwitchSubsystem limitSwitchSubsystem;

  public RetractIntakeWithElevator(ElevatorSubsystem elevatorSubsystem, LimitSwitchSubsystem limitSwitchSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.limitSwitchSubsystem = limitSwitchSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limitSwitchSubsystem.get()) {

    }
    else {

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
