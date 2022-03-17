// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class ArmsSubsystem extends SubsystemBase {
  //one motor controls both arms
  private CANSparkMax leftArm;
  private CANSparkMax rightArm;
  
  /** Creates a new ArmsSubsystem. */
  public ArmsSubsystem() {
    this.leftArm = new CANSparkMax(CanIds.armSpark1.id, MotorType.kBrushless); //temporary ID
    this.rightArm = new CANSparkMax(CanIds.armSpark2.id, MotorType.kBrushless);
    rightArm.setInverted(true);
    // brake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPower(double power) {
    leftArm.set(power);
    rightArm.set(power);
  }
  public void brake() {
    leftArm.setIdleMode(IdleMode.kBrake);
    rightArm.setIdleMode(IdleMode.kBrake);
  }
  public void coast() {
    leftArm.setIdleMode(IdleMode.kCoast);
    rightArm.setIdleMode(IdleMode.kCoast);
  }
}
