package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team7419.Initers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

/**
 * actual ball intake, as in like the rollers
 */
public class IntakeSub extends SubsystemBase{
    
    private VictorSPX victor;

    public IntakeSub(){
        victor = new VictorSPX(CanIds.intakeVictor.id);
        // victor.configFactoryDefault();
        Initers.initVictors(victor);
    }

    @Override
    public void periodic(){}

    public void setPower(double power){victor.set(ControlMode.PercentOutput, power);}
}