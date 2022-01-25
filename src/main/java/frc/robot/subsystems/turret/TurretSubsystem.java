// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team7419.TalonFuncs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIds;

public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax turret;
  private SparkMaxPIDController pidController;
  private RelativeEncoder turretEncoder;
  private double kP, kI, kD, kF, kMaxOutput, kMinOutput;

  private double rotations;

  private SparkMaxLimitSwitch forwardLimit;
  private SparkMaxLimitSwitch reverseLimit;

  public TurretSubsystem() {

    /* initialize turret talon */
    turret = new CANSparkMax(CanIds.turret.id, MotorType.kBrushless); // insert new CAN id for turret neo

    turret.restoreFactoryDefaults();

    pidController = turret.getPIDController();

    // Encoder object created to display position values
    turretEncoder = turret.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);;

    // set limit switches
    forwardLimit = turret.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    reverseLimit = turret.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

    forwardLimit.enableLimitSwitch(false);
    reverseLimit.enableLimitSwitch(false);

    pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", turretEncoder.getPosition());

    SmartDashboard.putBoolean("Forward Limit Enabled", forwardLimit.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Reverse Limit Enabled", reverseLimit.isLimitSwitchEnabled());
  }

  public void setPIDFConstants(double kP, double kD, double kI, double kF) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setFF(kF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void setPower(double power) {
    turret.set(ControlMode.PercentOutput, power);
  }

  public CANSparkMax getTurretMotor() {
    return turret;
  }
}
