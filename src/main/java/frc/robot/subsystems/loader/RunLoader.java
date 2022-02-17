// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RunLoader extends CommandBase {
  /** Creates a new RunLoader. */
  private LoaderSubsystem loaderSubsystem;
  private XboxController joystick;
  public RunLoader(LoaderSubsystem loaderSubsystem, XboxController joystick) {
    this.loaderSubsystem = loaderSubsystem;
    this.joystick = joystick;
    addRequirements(loaderSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getAButton()) {
      loaderSubsystem.setPower(0.3);
    }
    if (joystick.getYButton()) {
      loaderSubsystem.setPower(-0.3);
    }
    else {
      loaderSubsystem.setPower((0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
