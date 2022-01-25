package frc.robot.subsystems.pneumatics;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RunSolenoid extends CommandBase{
    private SolenoidSubsystem solenoidSubsystem;
    private XboxController joystick;


    public RunSolenoid(SolenoidSubsystem solenoidSubsystem, XboxController joystick) {
        this.solenoidSubsystem = solenoidSubsystem;
        this.joystick = joystick;
        addRequirements(solenoidSubsystem);
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (joystick.getXButtonPressed()) {
            solenoidSubsystem.actuateSolenoid();
            SmartDashboard.putBoolean("push", true);
        }
        else if (joystick.getYButtonPressed()){
            solenoidSubsystem.retractSolenoid();
            SmartDashboard.putBoolean("push", false);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return false;
    }

    


}