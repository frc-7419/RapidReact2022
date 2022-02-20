package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class IntakeSubsystem extends SubsystemBase{
    private CANSparkMax intakeMotor;
    private Solenoid intakeSolenoid; 

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(CanIds.intake.id, MotorType.kBrushless);
        intakeSolenoid = new Solenoid(1, PneumaticsModuleType.CTREPCM, 0);
    }

    @Override
    public void periodic() {}

    public void setPower(double power) {
        intakeMotor.set(power);
    }

    public void actuateSolenoid() {
        intakeSolenoid.set(true);
    }
    
    public void retractSolenoid() {
        intakeSolenoid.set(false);
    }

    public void toggleSolenoid() {
        intakeSolenoid.toggle();
    }

    public Solenoid getIntakeSolenoid() {
        return intakeSolenoid;
    }

    public CANSparkMax getIntakeMotor() {
        return intakeMotor;
    } 
}