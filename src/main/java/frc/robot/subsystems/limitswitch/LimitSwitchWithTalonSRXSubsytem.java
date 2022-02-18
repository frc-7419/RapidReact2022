// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limitswitch;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LimitSwitchWithTalonSRXSubsytem extends SubsystemBase {
  private TalonSRX talonSRX;
  
  /** Creates a new ElevatorSubsystem. */
  public LimitSwitchWithTalonSRXSubsytem() {
    talonSRX = new TalonSRX(50); //add the actual ID value
  }

  public TalonSRX getTalonSRX() {
    return talonSRX;
  }
  
  public void setPower(double power) {
    talonSRX.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
