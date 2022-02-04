package frc.robot.subsystems.ultrasonicSensor;

/* suggestion: import AnalogInput from wpilib, not revrobotics */
import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MaxBotixUltrasonicSensorSubsystem extends SubsystemBase {
    private final AnalogInput ultrasonic;
    double value;

    public MaxBotixUltrasonicSensorSubsystem() {
        ultrasonic = new AnalogInput(0);
        value = ultrasonic.getValue();
    }

    public double voltageToDistance() {
      double voltageDistance = 5/RobotController.getVoltage5V();
      return value * voltageDistance * 0.0492;
    }
  @Override 
  public void periodic() {
    SmartDashboard.putNumber("distance: ", voltageToDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
//test comment
}
