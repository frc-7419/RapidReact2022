// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Basic;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunShooterWithJoystick extends CommandBase {
  /** Creates a new RunShooterWithJoystick. */
  private BasicShooterSubsystem basicShooterSubsystem;
  private XboxController joystick;
  private double powerLeft;
  private double powerRight;
  private double powerBoth;

  public RunShooterWithJoystick(BasicShooterSubsystem basicShooterSubsystem, XboxController joystick) {
    this.basicShooterSubsystem = basicShooterSubsystem;
    this.joystick = joystick;
    addRequirements(basicShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      SmartDashboard.putNumber("power left", joystick.getLeftTriggerAxis());
      SmartDashboard.putNumber("power right", joystick.getRightTriggerAxis());

      powerLeft = joystick.getLeftTriggerAxis();
      powerRight = joystick.getRightTriggerAxis();
      basicShooterSubsystem.setTopPower(powerLeft);
      basicShooterSubsystem.setBottomPower(powerRight);

      if (joystick.getRightY() != 0) {
        basicShooterSubsystem.setBothPower(joystick.getRightY());
      }
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
