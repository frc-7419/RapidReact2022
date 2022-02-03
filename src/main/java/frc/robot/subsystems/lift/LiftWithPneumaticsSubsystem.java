package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pneumatics.SolenoidSubsystem;

public class LiftWithPneumaticsSubsystem extends SubsystemBase{
    private SolenoidSubsystem leftSolenoidSubsystem;
    private SolenoidSubsystem rightSolenoidSubsystem;

    
    public LiftWithPneumaticsSubsystem() {
        // need to change id and channel of the solenoids later
        this.leftSolenoidSubsystem = new SolenoidSubsystem(1,0); 
        this.rightSolenoidSubsystem = new SolenoidSubsystem(1,1);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    // actuate methods
    public void actuateLeft() {
        this.leftSolenoidSubsystem.actuateSolenoid();
    }

    public void actuateRight() {
        this.rightSolenoidSubsystem.actuateSolenoid();
    }

    // retract methods
    public void retractLeft() {
        this.leftSolenoidSubsystem.retractSolenoid();
    }

    public void retractRight() {
        this.rightSolenoidSubsystem.retractSolenoid();
    }
    
    // toggle methods
    public void toggleLeft() {
        this.leftSolenoidSubsystem.toggleSolenoid();
    }

    public void toogleRight() {
        this.rightSolenoidSubsystem.toggleSolenoid();
    }
}
