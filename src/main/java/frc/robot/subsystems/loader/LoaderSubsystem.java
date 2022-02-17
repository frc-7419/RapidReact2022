// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoaderSubsystem extends SubsystemBase {
  /** Creates a new LoaderSubsystem. */
  private TalonFX leftMotor;
  private TalonFX rightMotor;
  public LoaderSubsystem() {
    leftMotor = new TalonFX(0);
    rightMotor = new TalonFX(0);
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();
    leftMotor.setInverted(true);
    rightMotor.setInverted(false);
    leftMotor.setSensorPhase(false);
    rightMotor.setSensorPhase(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public TalonFX getRightMotor() {
    return rightMotor;
  }

  public TalonFX getLeftMotor() {
    return leftMotor;
  }
  
  public void setPower(double power) {
    leftMotor.set(ControlMode.PercentOutput, power);
    rightMotor.set(ControlMode.PercentOutput, power);
  }
  
}
