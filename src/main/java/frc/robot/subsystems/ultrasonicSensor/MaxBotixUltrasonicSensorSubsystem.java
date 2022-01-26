package frc.robot.subsystems.ultrasonicSensor;

import com.revrobotics.AnalogInput;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class MaxBotixUltrasonicSensorSubsystem {
    private AnalogInput ultrasonic;
    
    public MaxBotixUltrasonicSensorSubsystem() {
        ultrasonic = new AnalogInput(Constants.maxboticsUltrasonicId);
    }

    public void periodic() {
        SmartDashboard.putNumber("distance", getRawValue());
    }

    public double getRawValue() {
        return ultrasonic.getValue();
    }

    public double getInces() {
        return this.getRawValue() * 0.125;
    }
}
