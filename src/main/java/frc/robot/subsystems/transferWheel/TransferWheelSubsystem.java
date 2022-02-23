// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transferWheel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class TransferWheelSubsystem extends SubsystemBase {
  private VictorSPX transferWheel;

  public TransferWheelSubsystem() {
    transferWheel = new VictorSPX(CanIds.transferWheelVictor.id);
  }

  public void setPower(double power) {
    transferWheel.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
