// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spark;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxSubsystem extends SubsystemBase {
  /** Creates a new SparkMaxSoftLimit. */
  private CANSparkMax spark;
  // private Spark spark;

  public SparkMaxSubsystem() {
    spark = new CANSparkMax(21, MotorType.kBrushless);
    // spark = new Spark(0);
    // spark.restoreFactoryDefaults();
  }

  public void setPower(double power) {
    spark.set(power);
  }

  @Override
  public void periodic() {}

}
