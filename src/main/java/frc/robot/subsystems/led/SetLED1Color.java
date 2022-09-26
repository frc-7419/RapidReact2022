// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.led;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.drive.DriveBaseSubsystem;
// import frc.robot.subsystems.limelight.LimelightSubsystem;

// public class SetLED1Color extends CommandBase {
//   private LEDSubsystem ledSubsystem;
//   private DriveBaseSubsystem driveBaseSubsystem;
//   private int firstRainbowPixelHue = 0;

//   public SetLED1Color(LEDSubsystem ledSubsystem, DriveBaseSubsystem driveBaseSubsystem) {
//     this.ledSubsystem = ledSubsystem;
//     this.driveBaseSubsystem = driveBaseSubsystem;
//     addRequirements(ledSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     ledSubsystem.startLed();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     // purple
//     ledSubsystem.setLED1Color(255, 255, 0);
//     // ledSubsystem.stopLed();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }


// }
