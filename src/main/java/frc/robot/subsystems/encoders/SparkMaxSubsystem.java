// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxSubsystem extends SubsystemBase {
  /** Creates a new SparkMaxSubsystem. */
  private CANSparkMax canSparkMax;
  private SparkMaxLimitSwitch forwardLimitSwitch;
  private SparkMaxLimitSwitch reverseLimitSwitch;
  public SparkMaxSubsystem() {
    canSparkMax = new CANSparkMax(21, MotorType.kBrushless);
    forwardLimitSwitch = canSparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    reverseLimitSwitch = canSparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    
    forwardLimitSwitch.enableLimitSwitch(true);
    reverseLimitSwitch.enableLimitSwitch(true);

    // canSparkMax.burnFlash();
    
    SmartDashboard.putBoolean("Forward Limit Enabled", forwardLimitSwitch.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Reverse Limit Enabled", reverseLimitSwitch.isLimitSwitchEnabled());
  }
  public void setPower(double power) {
    canSparkMax.set(power);
  }

  public void brake() {
    canSparkMax.setIdleMode(IdleMode.kBrake);
  }
  public void coast() {
    canSparkMax.setIdleMode(IdleMode.kCoast);
  }

  public SparkMaxLimitSwitch getForwardLimitSwitch() {
    return forwardLimitSwitch;
  } 
  public SparkMaxLimitSwitch getReverseLimitSwitch() {
    return reverseLimitSwitch;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // enable/disable limit switches based on value read from SmartDashboard

    /**
     * The isPressed() method can be used on a SparkMaxLimitSwitch object to read the state of the switch.
     * 
     * In this example, the polarity of the switches are set to normally closed. In this case,
     * isPressed() will return true if the switch is pressed. It will also return true if you do not 
     * have a switch connected. isPressed() will return false when the switch is released.
     */
    SmartDashboard.putBoolean("Forward Limit Switch", forwardLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Reverse Limit Switch", reverseLimitSwitch.isPressed());
    
  }
}
