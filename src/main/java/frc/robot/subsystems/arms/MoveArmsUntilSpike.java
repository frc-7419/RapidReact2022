// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveArmsUntilSpike extends CommandBase {

  private ArmsSubsystem armsSubsystem;
  private double power;

  private final double CURRENT_THRESHOLD = 10;

  public MoveArmsUntilSpike(ArmsSubsystem armsSubsystem, double power) {
    this.armsSubsystem = armsSubsystem;
    this.power = power;
    addRequirements(armsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armsSubsystem.setPower(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armsSubsystem.setPower(0);
    armsSubsystem.zero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armsSubsystem.getCurrent() > CURRENT_THRESHOLD;
  }
}
