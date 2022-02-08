//period 6 file
package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

public class ShootGetSecondBallShoot extends SequentialCommandGroup {

    /* suggestions:
        remove the dependency injectin for straightwithmotionmagic and turnwithgyro, it's unnecessary
        you also need to inject GyroSubsystem into the constructor
        update:
        fixed!
    */
    public ShootGetSecondBallShoot(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) { //add parameters
        //shoot ball command
        addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, null, 180));
        addCommands(new WaitCommand(0.5));
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 120));
        //intake ball command
        addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 120));
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
