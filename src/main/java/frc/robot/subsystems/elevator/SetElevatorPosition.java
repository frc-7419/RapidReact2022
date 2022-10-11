// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetElevatorPosition extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private double pos;
  private double kp;
  private PIDController pidController;
  /** Creates a new SetElevatorPosition. */
  public SetElevatorPosition(ElevatorSubsystem elevatorSubsystem, double pos, double kp) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.pos = pos;
    this.kp = kp;
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kp = SmartDashboard.getNumber("elevatorkP", 0.0001);
    pos = SmartDashboard.getNumber("elevatorSetpoint", 4096);

    pidController = new PIDController(kp, 0, 0);
    pidController.setSetpoint(pos);
    pidController.setTolerance(0);

    elevatorSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(elevatorSubsystem.getElevatorPosition());
    SmartDashboard.putNumber("elevatorOutput", output);
    elevatorSubsystem.setPower(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
