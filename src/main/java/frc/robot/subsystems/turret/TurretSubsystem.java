// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax turret;
  private SparkMaxPIDController pidController;
  private RelativeEncoder turretEncoder;

  private SparkMaxLimitSwitch forwardLimit;
  private SparkMaxLimitSwitch reverseLimit;

  public TurretSubsystem() {
    // initialize turret talon
    turret = new CANSparkMax(CanIds.turret.id, MotorType.kBrushless); // insert new CAN id for turret neo

    turret.restoreFactoryDefaults();

    // Encoder object created to display position values
    turretEncoder = turret.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);

    // set encoder as feedback device for pid controller
    pidController.setFeedbackDevice(turretEncoder);

    // set PID controller
    pidController = turret.getPIDController();

    // set limit switches
    forwardLimit = turret.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    reverseLimit = turret.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

    forwardLimit.enableLimitSwitch(false);
    reverseLimit.enableLimitSwitch(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Position", turretEncoder.getPosition());

    SmartDashboard.putBoolean("Forward Limit Enabled", forwardLimit.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Reverse Limit Enabled", reverseLimit.isLimitSwitchEnabled());
  }

  public void setPIDFConstants(double kP, double kD, double kI, double kF) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setFF(kF);
  }

  public void setPower(double power) {
    turret.set(power);
  }

  public CANSparkMax getTurretMotor() {
    return turret;
  }

  public RelativeEncoder getTurretEncoder() {
    return turretEncoder;
  }

  public SparkMaxPIDController getTurretPIDController() {
    return pidController;
  }
}
