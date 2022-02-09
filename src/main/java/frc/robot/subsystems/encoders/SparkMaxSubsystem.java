// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxSubsystem extends SubsystemBase {
  /** Creates a new SparkMaxSubsystem. */
  private CANSparkMax canSparkMax;
  private DigitalInput limitSwitch;
  public SparkMaxSubsystem() {
    canSparkMax = new CANSparkMax(0, MotorType.kBrushless);
    limitSwitch = new DigitalInput(0);
    canSparkMax.restoreFactoryDefaults();
  }
  public DigitalInput getLimitSwitch() {
    return limitSwitch;
  }
  public boolean get() {
    return limitSwitch.get();
  }
  public void setSpeed(double speed) {
    canSparkMax.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
