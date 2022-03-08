// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax turret;

  private RelativeEncoder encoder;

  private SparkMaxLimitSwitch forwardLimitSwitch;
  private SparkMaxLimitSwitch reverseLimitSwitch;

  private double forwardLimitPosition = Double.MAX_VALUE; // rotations
  private double reverseLimitPosition = Double.MAX_VALUE;

  public TurretSubsystem() {
    turret = new CANSparkMax(CanIds.turretSpark.id, MotorType.kBrushless);
    
    forwardLimitSwitch = turret.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    reverseLimitSwitch = turret.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    encoder = turret.getEncoder();
    
    forwardLimitSwitch.enableLimitSwitch(false);
    reverseLimitSwitch.enableLimitSwitch(false);

    // turret.burnFlash();
  }

  @Override
  public void periodic() {
    if (getReverseLimitSwitch().isPressed() && reverseLimitPosition == Double.MAX_VALUE) {
      reverseLimitPosition = getEncoderPosition();
      SmartDashboard.putNumber("Reverse Limit Position", reverseLimitPosition);
    }
    if (getForwardLimitSwitch().isPressed() && forwardLimitPosition == Double.MAX_VALUE) {
      forwardLimitPosition = getEncoderPosition();
      SmartDashboard.putNumber("Forward Limit Position", forwardLimitPosition);
    }

    SmartDashboard.putBoolean("Forward Limit Switch", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimitSwitch.isPressed());
    SmartDashboard.putNumber("Turret Encoder Position", encoder.getPosition());
  }

  public void setPower(double power) {
    if (
      (getEncoderPosition() <= reverseLimitPosition && (power < 0 ) && (reverseLimitPosition != Double.MAX_VALUE)) ||
      (getEncoderPosition() >= forwardLimitPosition && (power > 0 ) && (forwardLimitPosition != Double.MAX_VALUE))
    ) {
      turret.set(0);
      brake();
    }
    else {
      turret.set(power);
      coast();
    }
  }

  public void brake() {
    turret.setIdleMode(IdleMode.kBrake);
  }
  public void coast() {
    turret.setIdleMode(IdleMode.kCoast);
  }

  public double getEncoderPosition() {
    return encoder.getPosition();
  }
  
  public SparkMaxLimitSwitch getForwardLimitSwitch() {
    return forwardLimitSwitch;
  } 
  public SparkMaxLimitSwitch getReverseLimitSwitch() {
    return reverseLimitSwitch;
  }

  public RelativeEncoder getTurretEncoder() {
    return encoder;
  }
}
