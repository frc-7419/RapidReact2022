// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spark;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickSparkMax extends CommandBase {
  /** Creates a new RunSparkMax. */
  private SparkMaxSubsystem sparkMaxSubsystem;
  private XboxController joystick;
  private double kSpeed;

  public JoystickSparkMax(SparkMaxSubsystem sparkMaxSubsystem, XboxController joystick, double kSpeed) {
    this.sparkMaxSubsystem = sparkMaxSubsystem;
    this.joystick = joystick;
    addRequirements(sparkMaxSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    sparkMaxSubsystem.setPower(kSpeed*joystick.getRightY());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
