// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class ReadEncoderValues extends CommandBase {
  /** Creates a new ReadEncoderValues. */
  private TalonFX talon;

  public ReadEncoderValues(TalonFX talon) {
    this.talon = talon;
    talon.configFactoryDefault();
    talon.setInverted(false);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Talon Position", talon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Talon Velocity", talon.getSelectedSensorVelocity());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
