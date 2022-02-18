// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limitswitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunLimitSwitchWithTalonSRX extends CommandBase {
  private LimitSwitchWithTalonSRXSubsytem limitSwitchWithTalonSRXSubsytem;
  private DigitalInput limitSwitch;
  private double power;

  public RunLimitSwitchWithTalonSRX(LimitSwitchWithTalonSRXSubsytem limitSwitchWithTalonSRXSubsytem, double power) {
    this.limitSwitchWithTalonSRXSubsytem = limitSwitchWithTalonSRXSubsytem;
    this.power = power;
    limitSwitch = new DigitalInput(1);
    
    addRequirements(limitSwitchWithTalonSRXSubsytem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limitSwitch.get()) {
      limitSwitchWithTalonSRXSubsytem.setPower(0);
    }
    else {
      limitSwitchWithTalonSRXSubsytem.setPower(power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limitSwitchWithTalonSRXSubsytem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
