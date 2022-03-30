// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunTurretWithJoystick extends CommandBase {
  /** Creates a new RunSparkMaxWithJoystick. */
  private TurretSubsystem turretSubsystem;
  private XboxController joystick;


  public RunTurretWithJoystick(TurretSubsystem turretSubsystem, XboxController joystick) {
    this.turretSubsystem = turretSubsystem;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getLeftY() != 0) {
      turretSubsystem.coast();
      turretSubsystem.setPower(joystick.getLeftY()*0.4);
    }
    else {
      turretSubsystem.setPower(0);
      turretSubsystem.brake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
