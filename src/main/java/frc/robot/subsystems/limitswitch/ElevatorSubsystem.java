// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limitswitch;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public TalonSRX elevatorLeft;
  public TalonSRX elevatorRight;


  public ElevatorSubsystem() {
    elevatorLeft = new TalonSRX(50);
    elevatorRight = new TalonSRX(51);
    elevatorRight.setInverted(true);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setElevatorPower(double power) {
    elevatorLeft.set(ControlMode.PercentOutput, power);
    elevatorRight.set(ControlMode.PercentOutput, power);
  }
}


