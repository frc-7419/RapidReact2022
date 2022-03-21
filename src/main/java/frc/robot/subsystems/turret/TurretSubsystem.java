// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX turret;

  public TurretSubsystem() {
    // initialize turret talon
    turret = new TalonFX(CanIds.turretFalcon.id); // insert new CAN id for turret neo
  }

  @Override
  public void periodic() {
  }

  public void setPower(double power) {
    turret.set(ControlMode.PercentOutput, power);
  }

  public void brake() {
    turret.setNeutralMode(NeutralMode.Brake);
  }

  public void coast() {
    turret.setNeutralMode(NeutralMode.Coast);
  }
}
