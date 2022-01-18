package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class ServoSubsystem extends SubsystemBase {
  private Servo servo;

  public ServoSubsystem() {
    servo = new Servo(0); // put the specific PWM port/channel
  }

  @Override
  public void periodic() {

  }

  // angle in degrees, from 0-180
  public void setAngle(double angle) {
    servo.setAngle(angle);
  }

  public double getAngle() {
    return servo.getAngle();
  }
}
