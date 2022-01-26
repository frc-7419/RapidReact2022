// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class MotionMagicTest extends CommandBase {
  /** Creates a new MotionMagicTest. */
  private TalonSubsystem talonSubsystem;
  private double setpoint;

  public MotionMagicTest(TalonSubsystem talonSubsystem, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.talonSubsystem = talonSubsystem;
    this.setpoint = setpoint;
    addRequirements(talonSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Status", "Starting Motion Magic Test");
    talonSubsystem.setTalonPosition(0);
    //0 is the timeoutMs
    talonSubsystem.getTalon().configMotionCruiseVelocity(15000,0);
    talonSubsystem.getTalon().configMotionCruiseVelocity(6000, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
