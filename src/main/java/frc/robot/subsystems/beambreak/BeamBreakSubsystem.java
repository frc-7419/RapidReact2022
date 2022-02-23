// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase {
  private DigitalInput beamBreak;

  /** Creates a new BeamBreakSubsystem. */
  public BeamBreakSubsystem() {
    beamBreak = new DigitalInput(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beamBreak: ", getBeamBreak());
    // This method will be called once per scheduler run
  }

  public DigitalInput getBeamBreak() {
    return beamBreak;
  }
  
  public boolean getBeamBreak(){
    return beamBreak.get();
  }

}
