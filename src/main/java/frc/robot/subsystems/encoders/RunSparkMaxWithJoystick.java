// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.encoders;

import java.sql.Time;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RunSparkMaxWithJoystick extends CommandBase {
  /** Creates a new RunSparkMaxWithJoystick. */
  private SparkMaxSubsystem sparkMaxSubsystem;
  private XboxController joystick;

  private double forwardLimitPosition = Double.MAX_VALUE; // rotations
  private double reverseLimitPosition = Double.MAX_VALUE;

  public RunSparkMaxWithJoystick(SparkMaxSubsystem sparkMaxSubsystem, XboxController joystick) {
    this.sparkMaxSubsystem = sparkMaxSubsystem;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sparkMaxSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (sparkMaxSubsystem.getReverseLimitSwitch().isPressed() && reverseLimitPosition == Double.MAX_VALUE) {
      reverseLimitPosition = sparkMaxSubsystem.getEncoderPosition();
      SmartDashboard.putNumber("Reverse Limit Position", reverseLimitPosition);
    }
    if (sparkMaxSubsystem.getForwardLimitSwitch().isPressed() && forwardLimitPosition == Double.MAX_VALUE) {
      forwardLimitPosition = sparkMaxSubsystem.getEncoderPosition();
      SmartDashboard.putNumber("Forward Limit Position", forwardLimitPosition);
    }

    if ((sparkMaxSubsystem.getEncoderPosition() <= reverseLimitPosition) && (joystick.getLeftY() < 0 ) && (reverseLimitPosition != Double.MAX_VALUE)) {
      sparkMaxSubsystem.setPower(0);
      sparkMaxSubsystem.brake();
    }
    else if ((sparkMaxSubsystem.getEncoderPosition() >= forwardLimitPosition) && (joystick.getLeftY() > 0 ) && (forwardLimitPosition != Double.MAX_VALUE)) {
      sparkMaxSubsystem.setPower(0);
      sparkMaxSubsystem.brake();
    }
    else {
      sparkMaxSubsystem.setPower(joystick.getLeftY()*0.15);
      sparkMaxSubsystem.coast();
    }

    // else {//sparkMaxSubsystem.getForwardLimitSwitch().isPressed() || sparkMaxSubsystem.getReverseLimitSwitch().isPressed()) {
      // sparkMaxSubsystem.brake();
      // SmartDashboard.putBoolean("is braking", true);
      // new WaitCommand(0.5);
      // sparkMaxSubsystem.coast();

    // }
    // else {
    //   sparkMaxSubsystem.setPower(joystick.getLeftY()*0.15);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sparkMaxSubsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
