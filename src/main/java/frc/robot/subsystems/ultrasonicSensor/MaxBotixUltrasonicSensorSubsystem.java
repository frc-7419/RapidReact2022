package frc.robot.subsystems.ultrasonicSensor;

import com.revrobotics.AnalogInput;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MaxBotixUltrasonicSensorSubsystem extends SubsystemBase {
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
