// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limitswitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.talon.TalonSubsystem;

public class RunLimitSwitchWithTalonSRX extends CommandBase {
  private LimitSwitchSubsystem limitSwitchSubsystem;
  private TalonSubsystem talonSubsystem;
  private DigitalInput limitSwitch;
  private double power;

  public RunLimitSwitchWithTalonSRX(LimitSwitchSubsystem limitSwitchSubsystem, double power, TalonSubsystem talonSubsystem) {
    this.limitSwitchSubsystem = limitSwitchSubsystem;
    this.talonSubsystem = talonSubsystem;
    this.power = power;
    limitSwitch = new DigitalInput(1);
    
    addRequirements(limitSwitchSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limitSwitch.get()) {
      talonSubsystem.setPower(0);
    }
    else {
      talonSubsystem.setPower(power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    talonSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
