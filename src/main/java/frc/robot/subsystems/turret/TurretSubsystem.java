// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.CanIds;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team7419.math.UnitConversions;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new  */
  private TalonFX turret;
  private DigitalInput forwardLimitSwitch;
  private DigitalInput reverseLimitSwitch;
  

  private boolean forwardLimitDetected = false;
  private boolean reverseLimitDetected = false;
  private double forwardLimitPosition;
  private double reverseLimitPosition;

  private static final double GEAR_RATIO = 99999;
  private static final double ANGLE_THRESHOLD = UnitConversions.degreesToTicks(30, GEAR_RATIO, 2048); // turret leeway extending limit switch; change 9999 to gear ratio

  public TurretSubsystem() {
    turret = new TalonFX(CanIds.turretFalcon.id);
    forwardLimitSwitch = new DigitalInput(1);
    reverseLimitSwitch = new DigitalInput(2);
    turret.configFactoryDefault();
    turret.configReverseSoftLimitEnable(false, 0);
    turret.configForwardSoftLimitEnable(false, 0);
  }

  @Override
  public void periodic() {
    // if (getReverseLimitSwitch().get() && !reverseLimitDetected) {
    //   reverseLimitDetected = true;
    //   turret.configReverseSoftLimitThreshold(turret.getSelectedSensorPosition(), 0);
    //   turret.configReverseSoftLimitEnable(true, 0);
    // } 
    // if (getForwardLimitSwitch().get() && !forwardLimitDetected) {
    //   forwardLimitDetected = true;
    //   turret.configForwardSoftLimitThreshold(turret.getSelectedSensorPosition(), 0);
    //   turret.configForwardSoftLimitEnable(true, 0);
    // } 
    // SmartDashboard.putBoolean("Reverse Limit Detected", reverseLimitDetected);

    // SmartDashboard.putBoolean("Forward Limit Detected", forwardLimitDetected);

    // SmartDashboard.putBoolean("Forward Limit Switch", forwardLimitSwitch.get());
    // SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimitSwitch.get());
    // SmartDashboard.putNumber("Turret Encoder Position", turret.getSelectedSensorPosition());
  }

  public void setPower(double power) {
    coast();
    turret.set(ControlMode.PercentOutput, power);
  }

  public void brake() {
    turret.setNeutralMode(NeutralMode.Brake);
  }
  public void coast() {
    turret.setNeutralMode(NeutralMode.Coast);
  }

  public DigitalInput getForwardLimitSwitch() {
    return forwardLimitSwitch;
  } 
  public DigitalInput getReverseLimitSwitch() {
    return reverseLimitSwitch;
  }

  public void setAngle(double angle) {
    // needs limit encoder pos as ref for 0, else quit
    if (!reverseLimitDetected || !forwardLimitDetected) return;
    //                      average of forward + reverse = 0                             add the desired angle
    double setpoint = (forwardLimitPosition + reverseLimitPosition)/2 + UnitConversions.degreesToTicks(angle, GEAR_RATIO, 2048);
    turret.set(ControlMode.MotionMagic, setpoint);
  }
}
