// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private TalonFX falcon;

  public TalonSubsystem() {
      falcon = new TalonFX(14);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public TalonFX getTalon() {
    return falcon;
  }

  public void setPower(double power) {
    falcon.set(ControlMode.PercentOutput, power);
  }

}
