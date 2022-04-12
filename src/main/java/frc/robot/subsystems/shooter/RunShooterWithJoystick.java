// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunShooterWithJoystick extends CommandBase {
  /** Creates a new RunShooterWithJoystick. */
  private ShooterSubsystem shooterSubsystem;
  private XboxController joystick;
  private double voltageTop = 0;
  private double voltageBottom = 0;
  private double stepInterval = 0.005;

  public RunShooterWithJoystick(ShooterSubsystem shooterSubsystem, XboxController joystick) {
    this.shooterSubsystem = shooterSubsystem;
    this.joystick = joystick;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  public static enum Direction {
    UP(0), RIGHT(90), DOWN(180), LEFT(270);

    int direction;

    private Direction(int direction) {
        this.direction = direction;
    }
}
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      int dPadValue = joystick.getPOV();
      if (dPadValue == Direction.UP.direction) {
        voltageTop += stepInterval;
      } else if (dPadValue == Direction.DOWN.direction) {
        voltageTop -= stepInterval;
      } else if (dPadValue == Direction.RIGHT.direction) {
        voltageBottom += stepInterval;
      } else if (dPadValue == Direction.LEFT.direction) {
        voltageBottom -= stepInterval;
      }
      shooterSubsystem.setTopVoltage(voltageTop);
      shooterSubsystem.setBottomVoltage(voltageBottom);
      // if (-joystick.getRightY() > 0) {
      //   shooterSubsystem.setBothvoltage(-joystick.getRightY());
        
      // }
      if (joystick.getBButtonPressed()) {
        shooterSubsystem.setBothVoltage(0);
        voltageTop = 0;
        voltageBottom = 0;
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
