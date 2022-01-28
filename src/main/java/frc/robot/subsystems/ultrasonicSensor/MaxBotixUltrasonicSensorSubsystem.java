package frc.robot.subsystems.ultrasonicSensor;

/* suggestion: import AnalogInput from wpilib, not revrobotics */
import com.revrobotics.AnalogInput;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MaxBotixUltrasonicSensorSubsystem extends SubsystemBase {
    private AnalogPotentiometer pot;
    private AnalogInput input;

    public MaxBotixUltrasonicSensorSubsystem() {
        //input = new AnalogInput(0); error --> cannot instanitate constructor
        //input.setAverageBits(2)
        pot = new AnalogPotentiometer(0,180,30);
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("distance: ", pot.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
