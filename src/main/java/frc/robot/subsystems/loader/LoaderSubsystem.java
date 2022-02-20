// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.loader;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class LoaderSubsystem extends SubsystemBase {
  private VictorSPX loader;

  public LoaderSubsystem() {
    loader = new VictorSPX(CanIds.loaderVictor.id);
    loader.configFactoryDefault();
    loader.setInverted(true);
    loader.setSensorPhase(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public VictorSPX getLoader() {
    return loader;
  }
  
  public void setPower(double power) {
    loader.set(ControlMode.PercentOutput, power);
  }
  
}
