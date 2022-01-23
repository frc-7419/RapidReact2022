// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private TalonFX turret;

  public TurretSubsystem() {
    turret = new TalonFX(5); // insert new CAN id for turret falcon
    
    /* Configured forward and reverse limit switch of Talon to be from a feedback connector and be normally open */
    turret.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    turret.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    turret.overrideLimitSwitchesEnable(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
