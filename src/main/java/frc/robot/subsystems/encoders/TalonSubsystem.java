// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class TalonSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private TalonFX talon;

  public TalonSubsystem(TalonFX talon) {
    this.talon = talon;

    talon.configFactoryDefault();
    talon.setInverted(false);
    talon.setSensorPhase(false);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }


}
