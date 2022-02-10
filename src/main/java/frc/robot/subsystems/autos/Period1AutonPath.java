package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

public class Period1AutonPath extends SequentialCommandGroup {

    public Period1AutonPath(DriveBaseSubsystem driveBase) {
        addCommands(new StraightWithMotionMagic(driveBase, 85.0));
        addCommands(new WaitCommand(1));
        addCommands(new StraightWithMotionMagic(driveBase, -85.0));
        
        //addCommands(commands);
        //Drive forward
        //Intake cargo
        //Drive backward
        //Shoot both cargo (inc. preload)
    }
}