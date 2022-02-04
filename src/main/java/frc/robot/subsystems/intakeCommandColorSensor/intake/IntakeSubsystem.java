package frc.robot.subsystems.intakeCommandColorSensor.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team7419.Initers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

/**
 * actual ball intake, as in like the rollers
 */
public class IntakeSubsystem extends SubsystemBase{
    
    private TalonFX talon;

    public IntakeSubsystem(){
        talon = new TalonFX(CanIds.intake.id);
    }

    @Override
    public void periodic(){}

    public TalonFX getTalon() {
        return talon;
    }

    public void setPower(double power) {
        talon.set(ControlMode.PercentOutput, power);
    }
}