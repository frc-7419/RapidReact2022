// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.limitswitch.LimitSwitchSubsystem;
import frc.robot.subsystems.pneumatics.SolenoidSubsystem;

public class PneumaticsRetractElevator extends CommandBase {
  /** Creates a new RetractIntakeWithElevator. */
  private LimitSwitchSubsystem elevatorLimitSwitchSubsystem;
  private SolenoidSubsystem solenoidSubsystem;
  private int status;

  public PneumaticsRetractElevator(LimitSwitchSubsystem elevatorLimitSwitchSubsystem, SolenoidSubsystem solenoidSubsystem, int status) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorLimitSwitchSubsystem = elevatorLimitSwitchSubsystem;
    this.solenoidSubsystem = solenoidSubsystem;
    this.status = status;
    addRequirements(elevatorLimitSwitchSubsystem, solenoidSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (status == 1) {
      if (elevatorLimitSwitchSubsystem.get()) {
        solenoidSubsystem.retractSolenoid();
      }
    }

    // else if (status == 2) {   // still need to implement
    //   if (intakeSubsystem.getPower() > 0) {
    //     intakeSubsystem.setPower(0);;
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
