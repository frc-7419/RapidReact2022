package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SolenoidForwardAndReverse extends SequentialCommandGroup {
    public SolenoidForwardAndReverse(SolenoidSubsystem solenoid) {
        addCommands(new RunPneumaticsSystem(solenoid, false));
        //addCommands(WaitCommand(0.1));
        addCommands(new RunPneumaticsSystem(solenoid, true));

    }
}
