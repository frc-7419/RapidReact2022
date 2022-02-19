// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

public class Period1ThreeBallClose extends SequentialCommandGroup {
  /** Creates a new Period1SecondAutonPath. */
  public Period1ThreeBallClose(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    /*
    // go forward to the second ball and intake
    // shoot the ball
    // travel to the third ball
    // go back into the tarmac and shoot
    // go back out of the tarmac
    */

    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 52)); 
    addCommands(new WaitCommand(0.2));
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -(180-58), PIDConstants.GyrokP122, PIDConstants.GyrokI122, PIDConstants.GyrokD122));// turn 58 degrees
    addCommands(new WaitCommand(0.2));
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 113));
    addCommands(new WaitCommand(0.2));
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -(180-58), PIDConstants.GyrokP122, PIDConstants.GyrokI122, PIDConstants.GyrokD122));// turn 58 degrees
    addCommands(new WaitCommand(0.2));
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 52));
  }

}
