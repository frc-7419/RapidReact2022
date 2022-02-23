// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunArmsWithJoystick extends CommandBase {
  private ArmsSubsystem armsSubsystem;
  private XboxController xboxController;
  /** Creates a new RunArmsWithJoystick. */
  public RunArmsWithJoystick(ArmsSubsystem armsSubsystem, XboxController xboxController) {
    this.armsSubsystem = armsSubsystem;
    this.xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.armsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //run with bumpers
    if (this.xboxController.getLeftBumperPressed()) {
      this.armsSubsystem.setPower(0.5);
    } else if (this.xboxController.getRightBumperPressed()) {
      this.armsSubsystem.setPower(-0.5);
    } else {
      this.armsSubsystem.setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.armsSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
