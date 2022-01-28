package frc.robot.subsystems.ultrasonicSensor;

/* suggestion: import AnalogInput from wpilib, not revrobotics */
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MaxBotixUltrasonicSensorSubsystem extends SubsystemBase {
    private AnalogInput input;
    private AnalogPotentiometer pot;
    
    public MaxBotixUltrasonicSensorSubsystem() {
        pot = new AnalogPotentiometer(3 ,180,30);
   
    }
    
    public void periodic() {
        SmartDashboard.putNumber("dist",pot.get());
    }
}