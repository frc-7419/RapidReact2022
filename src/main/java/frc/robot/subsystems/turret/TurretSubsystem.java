// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants.CanIds;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team7419.TalonFuncs;
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
    forwardLimitSwitch = new DigitalInput(0);
    reverseLimitSwitch = new DigitalInput(1);

    // because sample code 
    turret.configMotionCruiseVelocity(15000, 0);
    turret.configMotionAcceleration(6000, 0);
    turret.configMotionCruiseVelocity(15000, 0);
    turret.configMotionAcceleration(6000, 0);  
    //                                     P     I  D  F
    TalonFuncs.setPIDFConstants(0, turret, 0.01, 0, 0, 0);
    
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

  @Override
  public void periodic() {
    if (getReverseLimitSwitch().get() && !reverseLimitDetected) {
      reverseLimitDetected = true;
      reverseLimitPosition = turret.getSelectedSensorPosition();
      turret.configReverseSoftLimitThreshold(reverseLimitPosition-ANGLE_THRESHOLD, 0);
      turret.configReverseSoftLimitEnable(true, 0);
    } 
    if (getForwardLimitSwitch().get() && !forwardLimitDetected) {
      forwardLimitDetected = true;
      forwardLimitPosition = turret.getSelectedSensorPosition();
      turret.configForwardSoftLimitThreshold(forwardLimitPosition+ANGLE_THRESHOLD, 0);
      turret.configForwardSoftLimitEnable(true, 0);
    } 
    SmartDashboard.putBoolean("Reverse Limit Detected", reverseLimitDetected);

    SmartDashboard.putBoolean("Forward Limit Detected", forwardLimitDetected);

    SmartDashboard.putBoolean("Forward Limit Switch", forwardLimitSwitch.get());
    SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimitSwitch.get());
    SmartDashboard.putNumber("Turret Encoder Position", turret.getSelectedSensorPosition());
    
  }
}
