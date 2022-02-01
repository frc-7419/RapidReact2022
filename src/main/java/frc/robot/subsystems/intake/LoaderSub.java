package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

/**
 * wheel that pulls balls out of the lazy susan into the shooter
 */
public class LoaderSub extends SubsystemBase{
    
    private TalonFX falcon;

    public LoaderSub(){
        falcon = new TalonFX(CanIds.loaderFalcon.id);
        // falcon.configFactoryDefault();
    }

    @Override
    public void periodic(){}

    public void setPower(double power){falcon.set(ControlMode.PercentOutput, power);}

    public void brake() {
        falcon.setNeutralMode(NeutralMode.Brake);
    }
}