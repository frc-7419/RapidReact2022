// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.shooter.ShooterSubsystem;
public class RunShooterWithJoystick extends CommandBase {
  /** Creates a new RunShooterWithJoystick. */
  public ShooterSubsystem m_subsystem;
  public XboxController joystick;
  public double power;
  public double kPower = 1;
  public RunShooterWithJoystick(ShooterSubsystem subsystem, XboxController joystick) {
    m_subsystem = subsystem;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      power = kPower * joystick.getRightTriggerAxis();
      m_subsystem.setBothPower(power);
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
