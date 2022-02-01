package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team7419.MotorGroup;
import com.team7419.TalonFuncs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class ShooterSub extends SubsystemBase{

	public TalonFX talon;
    public MotorGroup motors;
    public double powerOutput = 0;
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0;
    public double targetVelocity = 0;
    public double target = 500;
    private double threshold = 200;
    public ControlMethod controlMethod = ControlMethod.PERCENT_OUTPUT;

    public ShooterSub(){

        talon = new TalonFX(CanIds.shooterFalcon.id);
        // talon.configFactoryDefault();
        talon.setInverted(true);
        talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    }

    public enum ControlMethod{
        PERCENT_OUTPUT,
        SPIN_UP,
        HOLDING,
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("periodic speed", talon.getSelectedSensorVelocity());
        // System.out.println(talon.getSelectedSensorVelocity());
    }

    public void run(){
        ControlMethod method = this.controlMethod;
        if(method == ControlMethod.PERCENT_OUTPUT){
            talon.set(ControlMode.PercentOutput, powerOutput);
        }
        else if(method == ControlMethod.HOLDING){
            talon.set(ControlMode.Velocity, target);
        }
        else if(method == ControlMethod.SPIN_UP){
            talon.set(ControlMode.Velocity, target);
        }

    }

    public void configureOutputs(){
        talon.configNominalOutputForward(0, 0);
		talon.configNominalOutputReverse(0, 0);
		talon.configPeakOutputForward(1, 0);
        talon.configPeakOutputReverse(-1, 0);
    }

    public void setPIDF(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        TalonFuncs.setPIDFConstants(0, talon, kP, kI, kD, kF);
    }

    public boolean onTarget(){
        return Math.abs(this.getCurrentRawSpeed() - target) < threshold;
    }

    public void setOutputPower(double power){this.powerOutput = power;}

    public void setkF(double kF){this.kF = kF;}

    public double getOutputVoltage(){
        return talon.getMotorOutputVoltage();
    }

    public void setTargetRpm(double rpm){this.target = rpm * 1.7067;}

    public void setControlMethod(ControlMethod method){
        this.controlMethod = method;
        if(method == ControlMethod.HOLDING){
            setPIDF(0,0,0,kF);
        }
    }

    public double lookUpkF(double nativeUnits){
        double output = 0;
        for(double[] pair : Constants.kSpeedToFf){
            if(pair[0] == nativeUnits){output = pair[1];}
        }
        if(output == 0){output = computekF(nativeUnits);}
        return output; 
    }

    public double computekF(double nativeUnits){
        return 47.3172/target + .0462152;
    }

    public double getCurrentRawSpeed(){return talon.getSelectedSensorVelocity(0);}

    public void setTargetRawSpeed(double speed){this.target = speed;}

    public void percentOutput(){
        talon.set(ControlMode.PercentOutput,powerOutput);
    }

    public void off(){
        talon.set(ControlMode.PercentOutput, 0);
    }

    public double getkP(){return kP;}
    public double getkI(){return kI;}
    public double getkD(){return kD;}
    public double getkF(){return kF;}

}