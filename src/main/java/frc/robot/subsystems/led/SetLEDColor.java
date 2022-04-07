// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetLEDColor extends CommandBase {
  /** Creates a new SetLEDColor. */
  private LEDSubsystem ledSubsystem;
  private int rainbowFirstPixelHue = 0;

  public SetLEDColor(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ledSubsystem.rainbowLED(rainbowFirstPixelHue);
    ledSubsystem.snakeRED(rainbowFirstPixelHue);
    ledSubsystem.startLed();
    rainbowFirstPixelHue += 3;
    // rainbowFirstPixelHue %= 180;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledSubsystem.stopLed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
