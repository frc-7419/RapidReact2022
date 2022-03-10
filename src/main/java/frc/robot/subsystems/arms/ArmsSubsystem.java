// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class ArmsSubsystem extends SubsystemBase {
  //one motor controls both arms
  private CANSparkMax armCAN;
  private CANSparkMax armCAN2;
  
  /** Creates a new ArmsSubsystem. */
  public ArmsSubsystem() {
    this.armCAN = new CANSparkMax(CanIds.armCAN.id, MotorType.kBrushless); //temporary ID
    this.armCAN2 = new CANSparkMax(CanIds.armCAN2.id, MotorType.kBrushless);
    // brake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power) {
    armCAN.set(power);
    armCAN2.set(-power);
  }
  public void brake() {
    armCAN.setIdleMode(IdleMode.kBrake);
    armCAN2.setIdleMode(IdleMode.kBrake);
  }
  public void coast() {
    armCAN.setIdleMode(IdleMode.kCoast);
    armCAN2.setIdleMode(IdleMode.kCoast);
  }
}
