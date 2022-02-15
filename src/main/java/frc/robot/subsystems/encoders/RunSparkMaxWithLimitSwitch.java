// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunSparkMaxWithLimitSwitch extends CommandBase {
  /** Creates a new RunSparkMaxWithLimitSwitch. */
  private SparkMaxSubsystem sparkMaxSubsystem;
  private XboxController joystick;
  public RunSparkMaxWithLimitSwitch(SparkMaxSubsystem sparkMaxSubsystem, XboxController joystick) {
    this.sparkMaxSubsystem = sparkMaxSubsystem;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(sparkMaxSubsystem);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Limit Switch", sparkMaxSubsystem.getLimitSwitch().get());
    if(!sparkMaxSubsystem.getLimitSwitch().get()) {
      sparkMaxSubsystem.setSpeed(joystick.getLeftX());
    }
    else if(sparkMaxSubsystem.getLimitSwitch().get()) {
      sparkMaxSubsystem.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sparkMaxSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
