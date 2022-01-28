package frc.robot.subsystems.ultrasonicSensor;

/* suggestion: import AnalogInput from wpilib, not revrobotics */
import com.revrobotics.AnalogInput;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MaxBotixUltrasonicSensorSubsystem extends SubsystemBase {
    private Ultrasonic ultrasonic;
    
    public MaxBotixUltrasonicSensorSubsystem() {
        ultrasonic = new Ultrasonic(5,6);
    }

    public void periodic() {
        SmartDashboard.putNumber("distance: ", ultrasonic.getRangeInches());
    }
}