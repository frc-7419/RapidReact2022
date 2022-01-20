// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunTalon extends CommandBase {
  private TalonSubsystem talonSubsystem;
  private XboxController joystick;
  /** Creates a new RunTalon. */
  public RunTalon(TalonSubsystem talonSubsystem, XboxController joystick) {
    this.talonSubsystem = talonSubsystem;
    this.joystick = joystick;
    addRequirements(talonSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    talonSubsystem.setPower(joystick.getRightY());
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
