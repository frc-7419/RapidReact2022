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
    
    private VictorSPX left;
    private VictorSPX right;
    private double power;

    public IntakeSubsystem(){
        left = new VictorSPX(CanIds.intakeVictor.id); //need to create two different can ids for each motor
        right = new VictorSPX(CanIds.intakeVictor.id);
        Initers.initVictors(left, right);
    }

    @Override
    public void periodic(){}

    public VictorSPX getLeft() {
        return left;
    }

    public VictorSPX getRight() {
      return right;
    }

    public void setPower(double power) {
        this.power = power;
        left.set(ControlMode.PercentOutput, power);
        right.set(ControlMode.PercentOutput, power);
    }

    public double getPower() {
        return this.power;
    }
}