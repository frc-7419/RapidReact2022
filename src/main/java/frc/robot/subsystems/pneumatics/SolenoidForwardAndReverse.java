package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SolenoidForwardAndReverse extends SequentialCommandGroup {
    public SolenoidForwardAndReverse(SolenoidSubsystem solenoid) {
        addCommands(new RunPneumaticsSystem(solenoid, false));
        addCommands(new WaitCommand(1));
        addCommands(new RunPneumaticsSystem(solenoid, true));

    }
}
