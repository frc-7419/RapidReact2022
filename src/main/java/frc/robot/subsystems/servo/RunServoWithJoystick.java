package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

public class RunServoWithJoystick extends CommandBase {
  private ServoSubsystem servoSubsystem;
  private XboxController joystick;
  private double angle1 = 0;
  private double angle2 = 120;

  public RunServoWithJoystick(ServoSubsystem servoSubsystem, XboxController joystick) {
    this.servoSubsystem = servoSubsystem;
    this.joystick = joystick;
    addRequirements(servoSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // toggle between init angle and target angle with Y and X button
    if (joystick.getYButton()) {
      servoSubsystem.setAngle(angle1);
    }
    if (joystick.getXButton()) {
      servoSubsystem.setAngle(angle2);
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
