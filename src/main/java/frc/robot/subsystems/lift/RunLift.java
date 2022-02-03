// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunLift extends CommandBase {
  /** Creates a new RunLift. */

  private LiftWithPneumaticsSubsystem liftWithPneumaticsSubsystem;
  private LiftWithSliderSubsystem liftWithSliderSubsystem;

  public RunLift(LiftWithPneumaticsSubsystem liftWithPneumaticsSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.liftWithPneumaticsSubsystem = liftWithPneumaticsSubsystem;
    this.liftWithSliderSubsystem = null;
    addRequirements(liftWithPneumaticsSubsystem);
  }

  public RunLift(LiftWithSliderSubsystem liftWithSliderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.liftWithSliderSubsystem = liftWithSliderSubsystem;
    this.liftWithPneumaticsSubsystem = null;
    addRequirements(liftWithSliderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (liftWithPneumaticsSubsystem != null) {
      liftWithPneumaticsSubsystem.actuateLeft();
      liftWithPneumaticsSubsystem.actuateRight();
    }
    else {
      liftWithSliderSubsystem.setLeftPower(10);
      liftWithSliderSubsystem.setRightPower(10);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (liftWithPneumaticsSubsystem != null) {
      liftWithPneumaticsSubsystem.retractLeft();
      liftWithPneumaticsSubsystem.retractRight();
    }
    else {
      liftWithSliderSubsystem.setLeftPower(0);
      liftWithSliderSubsystem.setRightPower(0);
    }    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
