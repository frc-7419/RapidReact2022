// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.revMagneticLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RevMagneticLimitSwitchSubsystem extends SubsystemBase {
  /** Creates a new RevMagneticLimitSwitchSubsystem. */
  DigitalInput revMagneticLimitSwitch;
  public RevMagneticLimitSwitchSubsystem() {
    this.revMagneticLimitSwitch = new DigitalInput(0);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("limit switch", revMagneticLimitSwitch.get());
  }

  public DigitalInput getRevMagneticLimitSwitch() {
    return revMagneticLimitSwitch;
}

public boolean get() {
  return revMagneticLimitSwitch.get();
}
}
