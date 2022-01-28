// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rev2mDistanceSensor;

import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rev2mDistanceSensorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private Rev2mDistanceSensor distanceSensor;
  
  public Rev2mDistanceSensorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
