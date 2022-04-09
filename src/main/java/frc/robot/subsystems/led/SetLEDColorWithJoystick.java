// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class SetLEDColorWithJoystick extends CommandBase {
  private XboxController joystick1;
  private XboxController joystick2;
  private LEDSubsystem ledSubsystem;
  private LimelightSubsystem limelightSubsystem;
  
  private int rainbowFirstPixelHue = 0;

  private double limelightTolerance = 7.5;

  public SetLEDColorWithJoystick(LEDSubsystem ledSubsystem, LimelightSubsystem limelightSubsystem, XboxController joystick1, XboxController joystick2) {
    this.joystick1 = joystick1;
    this.joystick2 = joystick2;
    this.ledSubsystem = ledSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(ledSubsystem);
    
    // purple
    ledSubsystem.setLEDColor(255, 0, 0);
    ledSubsystem.startLed();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledSubsystem.startLed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ledSubsystem.rainbowLED(rainbowFirstPixelHue);
    // ledSubsystem.startLed();
    // rainbowFirstPixelHue += 3;
    // rainbowFirstPixelHue %= 180;

    // while braking turret during hang
    if (joystick1.getBButton()) {
      // rainbow
      ledSubsystem.rainbowLED(rainbowFirstPixelHue);
      ledSubsystem.startLed();
      rainbowFirstPixelHue += 3;
      rainbowFirstPixelHue %= 180;
    } 
    else if (limelightSubsystem.getTv() == 1.0 && Math.abs(limelightSubsystem.getTx()) <= limelightTolerance && joystick2.getYButton()) {
      // green
      ledSubsystem.setLEDColor(0, 0, 255);
      // ledSubsystem.startLed();
    } 
    else if (limelightSubsystem.getTv() == 1.0 && Math.abs(limelightSubsystem.getTx()) <= limelightTolerance) {
      // blue
      ledSubsystem.setLEDColor(0, 255, 0);
      // ledSubsystem.startLed();
    } 
    else if (limelightSubsystem.getTv() == 1.0 && Math.abs(limelightSubsystem.getTx()) > limelightTolerance) {
      // yellow-ish white
      ledSubsystem.setLEDColor(255, 100, 150);
      // ledSubsystem.startLed();
    } 
    else {
      // red
      ledSubsystem.setLEDColor(255, 0, 0);
      // ledSubsystem.startLed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // purple
    ledSubsystem.setLEDColor(255, 255, 0);
    // ledSubsystem.stopLed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
