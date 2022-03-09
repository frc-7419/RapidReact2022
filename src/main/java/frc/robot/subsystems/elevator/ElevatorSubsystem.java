// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorLeft;
  private TalonFX elevatorRight;
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorLeft = new TalonFX(50);
    elevatorRight = new TalonFX(51);
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

  public void setPower(double power) {
    elevatorLeft.set(ControlMode.PercentOutput, power);
    elevatorRight.set(ControlMode.PercentOutput, power);
  }

  public void brake() {
    elevatorLeft.setNeutralMode(NeutralMode.Brake);
    elevatorRight.setNeutralMode(NeutralMode.Brake);
  }

  public void coast() {
    elevatorLeft.setNeutralMode(NeutralMode.Coast);
    elevatorRight.setNeutralMode(NeutralMode.Coast);
  }
}