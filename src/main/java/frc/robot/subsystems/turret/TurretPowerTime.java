// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurretPowerTime extends CommandBase {

  /** Creates a new TurretPowerTime. */
  private TurretSubsystem turretSubsystem;
  private double power;
  private double time;
  private double startTime;
  public TurretPowerTime(TurretSubsystem turretSubsystem,  double power, double time) {
    this.turretSubsystem = turretSubsystem;
    this.power= power;
    this.time = time;
    addRequirements(turretSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      turretSubsystem.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() - startTime > time){
        return true;
    } else {
        return false;
    }
  }
}
