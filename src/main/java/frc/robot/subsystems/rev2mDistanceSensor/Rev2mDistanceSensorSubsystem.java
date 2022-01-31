// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rev2mDistanceSensor;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rev2mDistanceSensorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private Rev2mDistanceSensor distanceSensor;
  
  public Rev2mDistanceSensorSubsystem() {
    this.distanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches, RangeProfile.kDefault);
  }

  public double getRange() {
    return distanceSensor.getRange();
  }

  public double getTimeStamp() {
    return distanceSensor.getTimestamp();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Range: ",getRange());
    SmartDashboard.putNumber("Timestamp: ",getTimeStamp());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
