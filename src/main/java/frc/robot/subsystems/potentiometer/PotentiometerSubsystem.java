// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.potentiometer;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PotentiometerSubsystem extends SubsystemBase {

  private AnalogPotentiometer arduinoPot;
  private AnalogPotentiometer vexPot;
  private double angle1;
  private double angle2;
  //30 is the starting point, 180 full range of motion

  public PotentiometerSubsystem() {
    arduinoPot = new AnalogPotentiometer(0, 360, 0);
    vexPot = new AnalogPotentiometer(1,180,0);
  }

  @Override
  public void periodic() {
    // This method will be called oence per scheduler run
    angle1 = arduinoPot.get();
    angle2 = vexPot.get();
    SmartDashboard.putNumber("arduinoPot: ", angle1);
    SmartDashboard.putNumber("vexPot", angle2);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
