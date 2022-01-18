//period 6 file
package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootGetSecondBallShoot extends SequentialCommandGroup {
    
    public ShootGetSecondBallShoot() { //add parameters
        addCommands(new WaitCommand(0.5)); //this is how the auton code should look: a series of addCommands() functions 
        //this is a comment
    }
}
