// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilIntaked extends CommandBase {

  private BeamBreakSubsystem beamBreakSubsystem;

  private int cargoToIntake;
  private int cargoIntaked = 0;
  private boolean intaking = false;

  /** Creates a new WaitUntilIntaked. */
  public WaitUntilIntaked(BeamBreakSubsystem beamBreakSubsystem, int cargoToIntake) {
    this.cargoToIntake = cargoToIntake;
    this.beamBreakSubsystem = beamBreakSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!intaking && beamBreakSubsystem.getIntakeBeamBreakActivated()) intaking = true;
    if (intaking && !beamBreakSubsystem.getIntakeBeamBreakActivated()) {
      intaking = false;
      cargoIntaked++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cargoIntaked == cargoToIntake;
  }
}
