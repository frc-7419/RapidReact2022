package frc.robot.subsystems.lift;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.subsystems.pneumatics.SolenoidSubsystem;

public class LiftWithSliderSubsystem extends SubsystemBase{
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    
    public LiftWithSliderSubsystem() {
        // need to change id and channel of the solenoids later
        this.leftMotor = new TalonFX(CanIds.leftLift.id);
        this.rightMotor = new TalonFX(CanIds.rightLift.id);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    public void setLeftPower(double power) {
        this.leftMotor.set(ControlMode.PercentOutput, power);
    }

    public void setRightPower(double power) {
        this.rightMotor.set(ControlMode.PercentOutput, power);
    }

    public void brake() {
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
    }

}
