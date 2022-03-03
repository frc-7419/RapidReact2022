// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase {
  private DigitalInput beamBreakReceiver;

  /** Creates a new BeamBreakSubsystem. */
  public BeamBreakSubsystem() { 
    beamBreakReceiver = new DigitalInput(5);

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beamBreak: ", beamBreakReceiver.get());
    // This method will be called once per scheduler run
  }

  public DigitalInput getBeamBreakReceiver() {
    return beamBreakReceiver;
  }
}
