package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

public class Period2Auton extends SequentialCommandGroup {

  public Period2Auton(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    // SmartDashboard.putString("command running", "thirty degrees");
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 30).withTimeout(2));
    // //addCommands(shootBall());
    SmartDashboard.putString("command running", "-70 deg");
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -70).withTimeout(5));
    SmartDashboard.putString("command running", "going back");
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, -40.375));
    // //addCommands(new collectBall());
    // //addCommands();
    SmartDashboard.putString("command running", "45 degrees");

    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -45).withTimeout(5));
    SmartDashboard.putString("command running", "going forward");
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 107));
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 60));
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, -60));


  }

}
