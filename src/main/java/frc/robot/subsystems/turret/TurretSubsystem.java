// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team7419.TalonFuncs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIds;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX turret;

  public TurretSubsystem() {

    /* initialize turret talon */
    turret = new TalonFX(CanIds.turretFalcon.id); // insert new CAN id for turret falcon
    turret.configFactoryDefault();
    // talon.setInverted(true);
    turret.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    
    /* Configured forward and reverse limit switch of Talon to be from a feedback connector and be normally open */
    turret.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    turret.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    turret.overrideLimitSwitchesEnable(true);

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Turret Reverse Limit Switch", turret.getSensorCollection().isRevLimitSwitchClosed() == 1);
    SmartDashboard.putBoolean("Turret Forward Limit Switch", turret.getSensorCollection().isFwdLimitSwitchClosed() == 1);
  }

  public void setPIDFConstants(double kP, double kD, double kI, double kF) {
    TalonFuncs.setPIDFConstants(0, turret, kP, kI, kD, kF);
  }

  public void setPower(double power) {
    turret.set(ControlMode.PercentOutput, power);
  }

  public TalonFX getTurretTalon() {
    return turret;
  }
}
