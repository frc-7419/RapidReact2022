// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxSubsystem extends SubsystemBase {
  /** Creates a new  */
  private CANSparkMax canSparkMax;
  private SparkMaxLimitSwitch forwardLimitSwitch;
  private SparkMaxLimitSwitch reverseLimitSwitch;

  private RelativeEncoder encoder;
  private boolean forwardLimitDetected = false;
  private boolean reverseLimitDetected = false;

  public SparkMaxSubsystem() {
    canSparkMax = new CANSparkMax(21, MotorType.kBrushless);
    forwardLimitSwitch = canSparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    reverseLimitSwitch = canSparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    encoder = canSparkMax.getEncoder();

    canSparkMax.restoreFactoryDefaults();
    
    forwardLimitSwitch.enableLimitSwitch(false);
    reverseLimitSwitch.enableLimitSwitch(false);



    // canSparkMax.burnFlash();

    SmartDashboard.putBoolean("Forward Limit Enabled", forwardLimitSwitch.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Reverse Limit Enabled", reverseLimitSwitch.isLimitSwitchEnabled());
  }
  public void setPower(double power) {
    coast();
    canSparkMax.set(power);
  }

  public void brake() {
    canSparkMax.setIdleMode(IdleMode.kBrake);
  }
  public void coast() {
    canSparkMax.setIdleMode(IdleMode.kCoast);
  }

  public SparkMaxLimitSwitch getForwardLimitSwitch() {
    return forwardLimitSwitch;
  } 
  public SparkMaxLimitSwitch getReverseLimitSwitch() {
    return reverseLimitSwitch;
  }


  @Override
  public void periodic() {
    if (getReverseLimitSwitch().isPressed() && !reverseLimitDetected) {
      reverseLimitDetected = true;
      canSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      canSparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)encoder.getPosition());
    } 
    if (getForwardLimitSwitch().isPressed() && !forwardLimitDetected) {
      forwardLimitDetected = true;
      canSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      canSparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)encoder.getPosition());

    } 
    SmartDashboard.putBoolean("Reverse Limit Detected", reverseLimitDetected);

    SmartDashboard.putBoolean("Forward Limit Detected", forwardLimitDetected);

    SmartDashboard.putBoolean("Forward Limit Switch", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimitSwitch.isPressed());
    // SmartDashboard.putNumber("Turret Encoder Position", encoder.getPosition());
    
  }
}
