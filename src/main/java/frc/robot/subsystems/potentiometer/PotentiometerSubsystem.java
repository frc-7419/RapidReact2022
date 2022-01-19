// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.potentiometer;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PotentiometerSubsystem extends SubsystemBase {

  private AnalogPotentiometer pot = new AnalogPotentiometer(0, 180, 30);
  private double angle;
  //30 is the starting point, 180 full range of motion

  public PotentiometerSubsystem() {
}

  @Override
  public void periodic() {
    // This method will be called oence per scheduler run
    angle = pot.get();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
