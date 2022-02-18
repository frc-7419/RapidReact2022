// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxSubsystem extends SubsystemBase {
  /** Creates a new SparkMaxSubsystem. */
  private Spark spark;
  private DigitalInput limitSwitch;
  public SparkMaxSubsystem() {
    spark = new Spark(0);
    limitSwitch = new DigitalInput(0);
  }
  public DigitalInput getLimitSwitch() {
    return limitSwitch;
  }
  public boolean get() {
    return limitSwitch.get();
  }
  public void setSpeed(double speed) {
    spark.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
