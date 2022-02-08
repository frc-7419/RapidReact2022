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

    private GyroSubsystem gyroSubsystem;
    public ShootGetSecondBallShoot(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, ReadyToShoot readyToShoot) { //add parameters
        //shoot ball command
        
        addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, null, 180));
        addCommands(new WaitCommand(0.5));
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 99.77));
        addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, null, 60));
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 172.8));
        addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, null, 30));
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 141.7));

        //intake ball command
        
        //shoot ball command
        






        //addCommands(new WaitCommand(0.5)); //this is how the auton code should look: a series of addCommands() functions
        //ReadyToShoot --> Load the ball into the shooter, basically get ready to shoot
        //RunShooter --> Actually shoot the ball. Power should be set accordingly for lower hub/upper hub
        //Turn robot to face one of the balls
        //Use limelight to find ball of our alliance color
        //Move Command to go to ball
        //Turn Command
        //ReadyToShoot
        //RunShooter


        //loadBallInShooter - load the next ball once we are ready to shoot
        //runShooter - shoot ball with desired power and at the target, make sure to account for whether we are shooting into upper or lower hub
        //adjust robot to face next ball to load
        //use limelight to find the desired ball of our alliance's color
        //use the move command to move the robot to the desired ball
        //use the turn command to position the robot for loading the ball
        //loadBallInShooter - load this ball into the shooter
        //use limelight to detect the hubs, reposition robot
        //runShooter - shoot ball with desired power and at the target, accounting for whether we are shooting into upper or lower hub


    }
}
