// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmsSubsystem extends SubsystemBase {
  //one motor controls both arms
  private TalonFX armTalon;
  /** Creates a new ArmsSubsystem. */
  public ArmsSubsystem() {
    this.armTalon = new TalonFX(11); //temporary ID
    brake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power) {
    armTalon.set(ControlMode.PercentOutput, power);
  }
  public void brake() {
    armTalon.setNeutralMode(NeutralMode.Brake);
  }
  public void coast() {
    armTalon.setNeutralMode(NeutralMode.Coast);
  }
}
