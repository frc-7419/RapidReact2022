//period 6 file
package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootGetSecondBallShoot extends SequentialCommandGroup {
    
    public ShootGetSecondBallShoot() { //add parameters
        addCommands(new WaitCommand(0.5)); //this is how the auton code should look: a series of addCommands() functions 
        //ReadyToShoot --> Load the ball into the shooter, basically get ready to shoot
        //RunShooter --> Actually shoot the ball. Power should be set accordingly for lower hub/upper hub
        //Turn robot to face one of the balls
        //Use limelight to find ball of our alliance color
        //Move Command to go to ball
        //Turn Command
        //ReadyToShoot
        //RunShooter
    }
}
