package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

public class Period2Auton extends SequentialCommandGroup {

  public Period2Auton(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 30));
    //addCommands(shootBall());
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -70));
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, -40.375));
    //addCommands(new collectBall());
    //addCommands();
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -45));
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 107));


  }

}
