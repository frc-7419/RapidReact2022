// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new  */
  private TalonFX talonFX;
  private DigitalInput forwardLimitSwitch;
  private DigitalInput reverseLimitSwitch;

  private boolean forwardLimitDetected = false;
  private boolean reverseLimitDetected = false;

  public TurretSubsystem() {
    //change later
    talonFX = new TalonFX(62);
    forwardLimitSwitch = new DigitalInput(1);
    reverseLimitSwitch = new DigitalInput(2);
    // canSparkMax.burnFlash();

  }
  public void setPower(double power) {
    coast();
    talonFX.set(ControlMode.PercentOutput, power);
  }

  public void brake() {
    talonFX.setNeutralMode(NeutralMode.Brake);
  }
  public void coast() {
    talonFX.setNeutralMode(NeutralMode.Coast);
  }

  public DigitalInput getForwardLimitSwitch() {
    return forwardLimitSwitch;
  } 
  public DigitalInput getReverseLimitSwitch() {
    return reverseLimitSwitch;
  }


  @Override
  public void periodic() {
    if (getReverseLimitSwitch().get() && !reverseLimitDetected) {
      reverseLimitDetected = true;
      talonFX.configReverseSoftLimitThreshold(talonFX.getSelectedSensorPosition(), 0);
      talonFX.configReverseSoftLimitEnable(true, 0);
    } 
    if (getForwardLimitSwitch().get() && !forwardLimitDetected) {
      forwardLimitDetected = true;
      talonFX.configForwardSoftLimitThreshold(talonFX.getSelectedSensorPosition(), 0);
      talonFX.configForwardSoftLimitEnable(true, 0);
    } 
    SmartDashboard.putBoolean("Reverse Limit Detected", reverseLimitDetected);

    SmartDashboard.putBoolean("Forward Limit Detected", forwardLimitDetected);

    SmartDashboard.putBoolean("Forward Limit Switch", forwardLimitSwitch.get());
    SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimitSwitch.get());
    SmartDashboard.putNumber("Turret Encoder Position", talonFX.getSelectedSensorPosition());
    
  }
}
