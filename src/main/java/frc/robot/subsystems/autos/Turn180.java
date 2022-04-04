// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Turn180 extends SequentialCommandGroup {

  public Turn180(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, 0.5, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180));
  }
}
