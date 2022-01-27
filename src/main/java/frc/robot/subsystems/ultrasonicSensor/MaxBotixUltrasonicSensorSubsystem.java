package frc.robot.subsystems.ultrasonicSensor;

/* suggestion: import AnalogInput from wpilib, not revrobotics */
import com.revrobotics.AnalogInput;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MaxBotixUltrasonicSensorSubsystem extends SubsystemBase {

    /* referencing the OffSeason code is fine, but i'd prefer if you look at documentation
    specific for the maxbotics sensor rather than c + p */

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
