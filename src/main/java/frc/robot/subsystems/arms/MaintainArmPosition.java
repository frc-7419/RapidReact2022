// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MaintainArmPosition extends CommandBase {
  private ArmsSubsystem armsSubsystem;
  private double kF;
  private double kP;
  private double setpoint;

  private PIDController pidController;

  public MaintainArmPosition(ArmsSubsystem armsSubsystem, double setpoint, double kP, double kF) {
    this.armsSubsystem = armsSubsystem;
    this.setpoint = setpoint;
    this.kP = kP;
    this.kF = kF;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = SmartDashboard.getNumber("armKp", 0.0001);
    setpoint = SmartDashboard.getNumber("armSetpoint", 2);

    pidController = new PIDController(kP, 0, 0);
    pidController.setSetpoint(setpoint);
    pidController.setTolerance(0);

    armsSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(armsSubsystem.getPosition()) + kF;
    armsSubsystem.setPower(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armsSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
