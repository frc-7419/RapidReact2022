// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.talon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class TalonSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private TalonFX talon;

  public TalonSubsystem() {
      talon = new TalonFX(0);
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
    return talon;
  }

  public void setPower(double power) {
    talon.set(ControlMode.PercentOutput, power);
  }

  public void setNeutralMode(NeutralMode brake) {
    talon.setNeutralMode(NeutralMode.Brake);
  }
  // write a method that puts the talon on brake mode, reference assignments for help

}
