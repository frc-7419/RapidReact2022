// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax turret;

  // private RelativeEncoder alternateEncoder;
  // private SparkMaxPIDController pidController;
  
  private SparkMaxLimitSwitch forwardLimit;
  private SparkMaxLimitSwitch reverseLimit;

  public TurretSubsystem() {
    // initialize turret talon
    turret = new CANSparkMax(CanIds.turretSpark.id, MotorType.kBrushless); // insert new CAN id for turret neo

    turret.restoreFactoryDefaults();

    // Encoder object created to display position values, 8192 counts per rev
    // alternateEncoder = turret.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

    // set PID controller
    // pidController = turret.getPIDController();

    // set encoder as feedback device for pid controller
    // pidController.setFeedbackDevice(alternateEncoder);

    // set limit switches
    forwardLimit = turret.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    reverseLimit = turret.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    forwardLimit.enableLimitSwitch(true);
    reverseLimit.enableLimitSwitch(true);

    turret.burnFlash();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Encoder Position", alternateEncoder.getPosition());

    // SmartDashboard.putBoolean("Forward Limit Enabled", forwardLimit.isLimitSwitchEnabled());
    // SmartDashboard.putBoolean("Reverse Limit Enabled", reverseLimit.isLimitSwitchEnabled());
  }

  // public void setPIDFConstants(double kP, double kD, double kI, double kF) {
  //   pidController.setP(kP);
  //   pidController.setI(kI);
  //   pidController.setD(kD);
  //   pidController.setFF(kF);
  // }

  public void setPower(double power) {
    turret.set(power);
  }

  public void brake() {
    turret.setIdleMode(IdleMode.kBrake);
  }

  public void coast() {
    turret.setIdleMode(IdleMode.kCoast);
  }

  public CANSparkMax getTurretMotor() {
    return turret;
  }

  // public RelativeEncoder getTurretEncoder() {
  //   return alternateEncoder;
  // }

  // public SparkMaxPIDController getTurretPIDController() {
  //   return pidController;
  // }
}
