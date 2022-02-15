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
        //Robot is initially facing the hub. We then shoot the ball. Next we will turn the robot so that it can go back
        //and collect the second ball and then shoot it
        addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180)); //180 degree turn. 
        //Decorator where if the command doesn't finish in that time interval it will move on
        addCommands(new WaitCommand(0.5));
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 67)); //The robot will ideally be positioned toward
        //the middle of the tarmac so it will have to move sttraight about half of the distance between the hub and the ball
        //to reach the ball
        addCommands(new WaitCommand(1));

        //Here, we will collect the ball and then turn around and then shoot it 
        addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180)); //180 degree turn
        addCommands(new WaitCommand(0.5));
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 67)); //The robot will ideally be positioned toward
        //here after moving 58.08 inches, it will return back to its orignal position and then shoot

        //changed the kD to 0.0001685

        
    }
}
