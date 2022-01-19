// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limitswitch;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//stephen was here
public class LimitSwitchSubsystem extends SubsystemBase{
  private DigitalInput limitSwitch;


  public LimitSwitchSubsystem(){
    limitSwitch = new DigitalInput(0);
  }

  public void periodic()
  {
      SmartDashboard.putBoolean("limit switch",limitSwitch.get());
  }
  
  public DigitalInput getLimitSwitch() {
      return limitSwitch;
  }

  

}
