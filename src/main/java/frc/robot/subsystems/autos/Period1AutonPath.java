package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
public class Period1AutonPath extends SequentialCommandGroup {

    public Period1AutonPath(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 85.0));
        addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180));
        
        //addCommands(commands);
        //Drive forward
        //Intake cargo
        //Drive backward
        //Shoot both cargo (inc. preload)
    }
}