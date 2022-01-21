package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunPneumaticsSystem extends CommandBase{
    private final SolenoidSub solenoid;
    private boolean reversed;


    public RunPneumaticsSystem(SolenoidSub solonoid, boolean reversed) {
        this.solenoid = solonoid;
        this.reversed = reversed;
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (this.reversed == true) {
            solenoid.retractSolenoid();
        }
        else {
            solenoid.actuateSolenoid();
        }
    }

    @Override
    public void end(boolean interrupted) {
        solenoid.stopSolenoid();
    }

    public boolean isFinished() {
        return false;
    }

    


}