// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  Compressor c;
  boolean enabled = true;
  boolean pressureSwitch;
  double current;

  public CompressorSubsystem() {
    c = new Compressor(0, PneumaticsModuleType.CTREPCM);
    c.enableDigital();
    pressureSwitch = c.getPressureSwitchValue();
    current = c.getCurrent();
  }

  public void start() { 
    c.enableDigital();
  }

  public void stop() {
    c.disable();
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
