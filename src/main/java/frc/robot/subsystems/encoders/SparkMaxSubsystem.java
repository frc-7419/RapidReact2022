// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxSubsystem extends SubsystemBase {
  /** Creates a new SparkMaxSoftLimit. */
  private CANSparkMax spark;

  public SparkMaxSubsystem() {
    spark = new CANSparkMax(21, MotorType.kBrushless);
    spark.restoreFactoryDefaults();
    spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    spark.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    spark.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    spark.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
  }

  public void setSpeed(double power) {
    spark.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getSoftLimitForward() {
    SmartDashboard.putNumber("forward: ", spark.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
  }
  public void getSoftLimitReverse() {
    SmartDashboard.putNumber("forward: ", spark.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
  }
}
