package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

private Servo servo;

public class ServoSubsystem extends SubsystemBase {
  

  public ServoSubsystem() {
    servo = new Servo(0); // put the specific PWM
  }

  @Override
  public void periodic() {

  }

  // Servo motors are controlled on a scale of 0.0 to 1.0
  public void setAngle(double angle) {
    servo.set(angle);
  }
}
