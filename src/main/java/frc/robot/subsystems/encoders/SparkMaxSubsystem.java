// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxSubsystem extends SubsystemBase {
  /** Creates a new SparkMaxSubsystem. */
  private CANSparkMax canSparkMax;
  private SparkMaxLimitSwitch forwardLimitSwitch;
  private SparkMaxLimitSwitch reverseLimitSwitch;
  private XboxController joystick;
  public SparkMaxSubsystem() {
    canSparkMax = new CANSparkMax(21, MotorType.kBrushless);
    forwardLimitSwitch = canSparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    reverseLimitSwitch = canSparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    canSparkMax.restoreFactoryDefaults();
    forwardLimitSwitch.enableLimitSwitch(false);
    reverseLimitSwitch.enableLimitSwitch(false);
    SmartDashboard.putBoolean("Forward Limit Enabled", forwardLimitSwitch.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Reverse Limit Enabled", reverseLimitSwitch.isLimitSwitchEnabled());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
