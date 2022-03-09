// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase {
  private DigitalInput intakeBeamBreakReceiver;
  private DigitalInput shooterBeamBreakReceiver;

  /** Creates a new BeamBreakSubsystem. */
  public BeamBreakSubsystem() { 
    intakeBeamBreakReceiver = new DigitalInput(5);
    shooterBeamBreakReceiver = new DigitalInput(7); // change this
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("intakeBeamBreak: ", intakeBeamBreakReceiver.get());
    // This method will be called once per scheduler run
  }

  // returns true when nothing is between sensors
  public boolean getIntakeBeamBreakActivated() {
    return intakeBeamBreakReceiver.get();
  }
  public boolean getShooterBeamBreakActivated() {
    return shooterBeamBreakReceiver.get();
  }

  // public DigitalInput getBeamBreakReceiver() {
  //   return beamBreakReceiver;
  // }
}
