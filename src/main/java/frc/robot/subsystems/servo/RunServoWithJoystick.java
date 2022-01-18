package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

public class RunServoWithJoystick extends CommandBase {
  private ServoSubsystem servoSubsystem;
  private XboxController joystick;
  private double initAngle;
  private double targetAngle;

  public RunServoWithJoystick(ServoSubsystem servoSubsystem, XboxController joystick, double targetAngle) {
    this.servoSubsystem = servoSubsystem;
    this.joystick = joystick;
    this.targetAngle = targetAngle;
    addRequirements(servoSubsystem);
  }

  @Override
  public void initialize() {
    // angle that servo starts at
    initAngle = servoSubsystem.getAngle();
  }

  @Override
  public void execute() {
    // toggle between init angle and target angle with Y button
    if (joystick.getYButton()) {
      if (servoSubsystem.getAngle() == initAngle) {
        servoSubsystem.setAngle(targetAngle);
      }
      else {
        servoSubsystem.setAngle(initAngle);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
