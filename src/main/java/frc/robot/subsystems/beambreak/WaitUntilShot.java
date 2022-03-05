// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilShot extends CommandBase {

  private BeamBreakSubsystem beamBreakSubsystem;

  private int cargoToShoot;
  private int cargoShot = 0;
  private boolean shooting = false;

  /** Creates a new WaitUntilIntaked. */
  public WaitUntilShot(BeamBreakSubsystem beamBreakSubsystem, int cargoToShoot) {
    this.cargoToShoot = cargoToShoot;
    this.beamBreakSubsystem = beamBreakSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooting && beamBreakSubsystem.getShooterBeamBreakActivated()) shooting = true;
    if (shooting && !beamBreakSubsystem.getShooterBeamBreakActivated()) {
      shooting = false;
      cargoShot++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cargoShot == cargoToShoot;
  }
}
