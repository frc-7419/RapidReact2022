package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

public class Period2Auton extends SequentialCommandGroup {

  public Period2Auton(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    //Turns robot towards the hub
    //addCommands(shootBall());
    SmartDashboard.putString("command running", "161 degrees");
    //addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 161, PIDConstants.GyrokP161, PIDConstants.GyrokI161, PIDConstants.GyrokD161));
    SmartDashboard.putString("command running", "going forward");
    //addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 40.375));
    // //addCommands(new collectBall());
    SmartDashboard.putString("command running", "115 degrees");
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 115, PIDConstants.GyrokP115, PIDConstants.GyrokI115, PIDConstants.GyrokD115));
    SmartDashboard.putString("command running", "going forward");
    //addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 107));
    //addCommands(new collectBall());
    //addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 80, PIDConstants.GyrokP80, PIDConstants.GyrokI80, PIDConstants.GyrokD80));
    //addCommands(new shootBall());


  }

}
