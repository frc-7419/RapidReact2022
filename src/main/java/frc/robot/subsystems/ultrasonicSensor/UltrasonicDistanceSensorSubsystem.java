// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ultrasonicSensor;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicDistanceSensorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private Ultrasonic ultrasonic;
  public UltrasonicDistanceSensorSubsystem() {
    ultrasonic = new Ultrasonic(2, 3);
    Ultrasonic.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ultrasonic Sensor Distance", ultrasonic.getRangeInches());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
