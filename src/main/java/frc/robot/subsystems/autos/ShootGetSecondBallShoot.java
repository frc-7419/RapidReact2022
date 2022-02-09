//period 6 file
package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

public class ShootGetSecondBallShoot extends SequentialCommandGroup {

    /* comments from Karan:
    you dont need to declare GyroSubsystem since its alr being injected
    we're not using ReadyToShooter, remove it from the constructor, it wont let your code build
    instead of 'null' for the gyro command, substitute it with your instance of GyroSubsystem
    */

    public ShootGetSecondBallShoot(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) { //add parameters
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 36));
        addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 90));
        
    }
}
