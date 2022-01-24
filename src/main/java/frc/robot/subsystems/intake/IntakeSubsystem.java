package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team7419.Initers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

/**
 * actual ball intake, as in like the rollers
 */
public class IntakeSubsystem extends SubsystemBase{
    
    private VictorSPX intake;

    public IntakeSubsystem(){
        intake = new VictorSPX(CanIds.intakeVictor.id);
        Initers.initVictors(intake);
    }

    @Override
    public void periodic(){}

    public VictorSPX getIntake() {
        return intake;
    }

    public void setPower(double power) {
        intake.set(ControlMode.PercentOutput, power);
    }
}