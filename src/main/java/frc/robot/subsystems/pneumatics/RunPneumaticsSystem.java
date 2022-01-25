package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunPneumaticsSystem extends CommandBase{
    private SolenoidSubsystem solenoidSubsystem;
    private boolean reversed;


    public RunPneumaticsSystem(SolenoidSubsystem solenoidSubsystem, boolean reversed) {
        this.solenoidSubsystem = solenoidSubsystem;
        this.reversed = reversed;
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (this.reversed == true) {
            solenoidSubsystem.retractSolenoid();
        }
        else {
            solenoidSubsystem.actuateSolenoid();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return false;
    }

    


}