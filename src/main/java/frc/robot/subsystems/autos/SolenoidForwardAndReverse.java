package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.pneumatics.PneumaticsSub;
import frc.robot.subsystems.pneumatics.RunPneumaticsSystem;
import frc.robot.subsystems.pneumatics.SolenoidSub;

public class SolenoidForwardAndReverse extends SequentialCommandGroup {
    public SolenoidForwardAndReverse(SolenoidSub solenoid) {
        addCommands(new RunPneumaticsSystem(solenoid, false));
        //addCommands(WaitCommand(0.1));
        addCommands(new RunPneumaticsSystem(solenoid, true));

    }
}
