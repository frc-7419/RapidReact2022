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
    SmartDashboard.putString("command running", "thirty degrees");
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 30, PIDConstants.GyrokP30, PIDConstants.GyrokI30, PIDConstants.GyrokD30).withTimeout(2));
    // //addCommands(shootBall());
    SmartDashboard.putString("command running", "-49 deg");
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -49, PIDConstants.GyrokPNegative49, PIDConstants.GyrokINegative49, PIDConstants.GyrokDNegative49).withTimeout(5));
    SmartDashboard.putString("command running", "going back");
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, -40.375));
    // //addCommands(new collectBall());
    // //addCommands();
    SmartDashboard.putString("command running", "-49 degrees");

    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 115, PIDConstants.GyrokP115, PIDConstants.GyrokI115, PIDConstants.GyrokD115).withTimeout(2));
    SmartDashboard.putString("command running", "going forward");
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 107));
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 80, PIDConstants.GyrokP80, PIDConstants.GyrokI80, PIDConstants.GyrokD80).withTimeout(5));


  }

}
