// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

public class Period1ThreeBallClose extends SequentialCommandGroup {
  /** Creates a new Period1SecondAutonPath. */
  public Period1ThreeBallClose(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 52));
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 58, PIDConstants.GyrokP58, PIDConstants.GyrokI58, PIDConstants.GyrokD58));
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 113));
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 58, PIDConstants.GyrokP58, PIDConstants.GyrokI58, PIDConstants.GyrokD58));
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 52));
  }

}
