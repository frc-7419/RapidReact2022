// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private TalonSRX talon1;
  private TalonSRX talon2;
  
  public TalonSubsystem() {
    talon1 = new TalonSRX(50);
    //talon2 = new TalonSRX(48); for a second motor 
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setPower(double power) {
    talon1.set(ControlMode.PercentOutput, power);
    //talon2.set(ControlMode.PercentOutput, power); for a second motor

  }

}
