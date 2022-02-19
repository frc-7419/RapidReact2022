package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
public class Period1ThreeBallAutonFar extends SequentialCommandGroup {

    public Period1ThreeBallAutonFar(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 75.0));
        addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 135, PIDConstants.GyrokP115, PIDConstants.GyrokI115, PIDConstants.GyrokD115));
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 116.4));
        addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 63, PIDConstants.GyrokP63, PIDConstants.GyrokP63, PIDConstants.GyrokP63));
        

// if its not reaching setpoint, increase P
// if its overshooting, increase D
// increase I if it overshoots and doesn't get back fast enough

        //straight to second ball
        //turn left to face third ball
        //turn turret and shoot preload and second ball
        //forward to third ball
        //turn around 45-90 degrees left to make room for turret to turn and shoot
    }
}