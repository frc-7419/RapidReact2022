// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArrowRunShooterWithJoystick extends CommandBase {
  /** Creates a new RunShooterWithJoystick. */
  private BasicShooterSubsystem basicShooterSubsystem;
  private XboxController joystick;
  private double powerTop = 0;
  private double powerBottom = 0;

  public ArrowRunShooterWithJoystick(BasicShooterSubsystem basicShooterSubsystem, XboxController joystick) {
    this.basicShooterSubsystem = basicShooterSubsystem;
    this.joystick = joystick;
    addRequirements(basicShooterSubsystem);
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
      SmartDashboard.putNumber("power top", powerTop);
      SmartDashboard.putNumber("power bottom", powerBottom);
      int dPadValue = joystick.getPOV();
      if (dPadValue == Direction.UP.direction) {
        powerTop += 0.025;
      } else if (dPadValue == Direction.DOWN.direction) {
        powerTop -= 0.025;
      } else if (dPadValue == Direction.RIGHT.direction) {
        powerBottom += 0.025;
      } else if (dPadValue == Direction.LEFT.direction) {
        powerBottom -= 0.025;
      }
      basicShooterSubsystem.setTopPower(powerTop);
      basicShooterSubsystem.setBottomPower(powerBottom);
      if (joystick.getRightY() != 0) {
        basicShooterSubsystem.setBothPower(joystick.getRightY());
      }
      if (joystick.getBButtonPressed()) {
        basicShooterSubsystem.setBothPower(0);
        powerTop = 0;
        powerBottom = 0;
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
