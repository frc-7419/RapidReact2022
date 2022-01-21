// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.potentiometer;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PotentiometerSubsystem extends SubsystemBase {

  private AnalogPotentiometer potentiometer;
  private double angle;
  //30 is the starting point, 180 full range of motion

  public PotentiometerSubsystem() {
    potentiometer = new AnalogPotentiometer(1,180,0);
  }

  @Override
  public void periodic() {
    // This method will be called oence per scheduler run
    angle = potentiometer.get();
    SmartDashboard.putNumber("potentiometer: ", angle);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
