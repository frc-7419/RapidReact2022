// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JoystickSparkMax extends CommandBase {
  /** Creates a new RunSparkMax. */
  private SparkMaxSubsystem sparkMaxSubsystem;
  private XboxController joystick;

  public JoystickSparkMax(SparkMaxSubsystem sparkMaxSubsystem, XboxController joystick) {
    this.sparkMaxSubsystem = sparkMaxSubsystem;
    this.joystick = joystick;
    addRequirements(sparkMaxSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sparkMaxSubsystem.setPower(joystick.getRightY());
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
