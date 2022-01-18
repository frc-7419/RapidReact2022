package frc.robot.subsystems.servo;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

private Servo servo;

public class ServoSubsystem extends SubsystemBase {
  

  public ServoSubsystem() {
    servo = new Servo(0); // put the specific PWM port/channel
  }

  @Override
  public void periodic() {

  }

  // angle in degrees
  public void setAngle(double angle) {
    servo.setAngle(angle);
  }

  public double getAngle() {
    return servo.getAngle();
  }
}
