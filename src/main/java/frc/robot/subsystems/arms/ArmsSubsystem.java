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
  private CANSparkMax armSpark1;
  private CANSparkMax armSpark2;
  
  /** Creates a new ArmsSubsystem. */
  public ArmsSubsystem() {
    this.armSpark1 = new CANSparkMax(CanIds.armCAN.id, MotorType.kBrushless); //temporary ID
    this.armSpark2 = new CANSparkMax(CanIds.armCAN2.id, MotorType.kBrushless);
    armSpark2.setInverted(true);
    // brake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power) {
    armSpark1.set(power);
    armSpark2.set(power);
  }
  public void brake() {
    armSpark1.setIdleMode(IdleMode.kBrake);
    armSpark2.setIdleMode(IdleMode.kBrake);
  }
  public void coast() {
    armSpark1.setIdleMode(IdleMode.kCoast);
    armSpark2.setIdleMode(IdleMode.kCoast);
  }
}
