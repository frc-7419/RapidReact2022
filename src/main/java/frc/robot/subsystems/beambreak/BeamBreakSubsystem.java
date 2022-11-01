// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BeamBreakSubsystem extends SubsystemBase {
  private DigitalInput beamBreakReceiver;
  private int detections = 0;
  private Long startTime;
  /** Creates a new BeamBreakSubsystem. */
  public BeamBreakSubsystem() { 
    beamBreakReceiver = new DigitalInput(5);
    startTime = System.currentTimeMillis();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beamBreak: ", beamBreakReceiver.get());
    // WaitCommand(10);
    // // This method will be called once per scheduler run

    // if (detections < 2) {
    //   if (beamBreakReceiver.get() == true) {
    //     detections++;
    //   }
    // } else {
    //   WaitCommand(10);
    //   detections -= 2;
    // } 
    long endTime = System.currentTimeMillis();

    if (detections >= 0 && detections <= 2) {
      if (getBeamBreakActivated()) {
        detections++;
      }
      else {
        detections--;
      }

      if (endTime-startTime == 10000){
        startTime = endTime;
        detections = 0;
      }
    }
  }

  public boolean getBeamBreakActivated() {
    return beamBreakReceiver.get();
  }

  public DigitalInput getBeamBreakReceiver() {
    return beamBreakReceiver;
  }
}
