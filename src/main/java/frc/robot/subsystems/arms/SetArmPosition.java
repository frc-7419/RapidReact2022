// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmPosition extends CommandBase {
  /** Creates a new SetPosition. */
  private ArmsSubsystem armSubsystem;
  private double pos;
  private double kP;
  private PIDController pidController;

  public SetArmPosition(ArmsSubsystem armSubsystem, double pos, double kP) {
    this.armSubsystem = armSubsystem;
    this.pos = pos;
    this.kP = kP;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController = new PIDController(kP, 0, 0);
    pidController.setSetpoint(pos);
    pidController.setTolerance(0.1);

    armSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(armSubsystem.getPosition());
    armSubsystem.setPower(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
