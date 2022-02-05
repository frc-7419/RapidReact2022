package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team7419.TalonFuncs;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX bottomFalcon;
    private TalonFX topFalcon;
    private double kF;
    private double powerOutput = 0;
    private double bottomTargetVelocity = 500;
    private double topTargetVelocity = 500;
    private double velocityThreshold = 200;
    private ControlMethod controlMethod = ControlMethod.PERCENT_OUTPUT;

    public ShooterSubsystem(){
        bottomFalcon = new TalonFX(CanIds.bottomShooterFalcon.id);
        topFalcon = new TalonFX(CanIds.topShooterFalcon.id);

        // bottomFalcon.configFactoryDefault();
        // topFalcon.configFactoryDefault();

        bottomFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        topFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        bottomFalcon.setInverted(true);
        topFalcon.setInverted(false);
        
        configShooterOutputs();
    }

    public enum ControlMethod {
        PERCENT_OUTPUT,
        SPIN_UP,
        HOLDING,
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("TF RS", topFalcon.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("BF RS", bottomFalcon.getSelectedSensorVelocity(0));

        SmartDashboard.putNumber("TF RPM", UnitConversions.rawSensorVelocityToRPM(topFalcon.getSelectedSensorVelocity(0), 2048));
        SmartDashboard.putNumber("BF RPM", UnitConversions.rawSensorVelocityToRPM(bottomFalcon.getSelectedSensorVelocity(0), 2048));

        SmartDashboard.putNumber("TF RPM Graph", UnitConversions.rawSensorVelocityToRPM(topFalcon.getSelectedSensorVelocity(0), 2048));
        SmartDashboard.putNumber("BF RPM Graph", UnitConversions.rawSensorVelocityToRPM(bottomFalcon.getSelectedSensorVelocity(0), 2048));
    }

    public void run() {
        ControlMethod method = this.controlMethod;
        if (method == ControlMethod.PERCENT_OUTPUT){
            topFalcon.set(ControlMode.PercentOutput, powerOutput);
            bottomFalcon.set(ControlMode.PercentOutput, powerOutput);
        }
        else if (method == ControlMethod.HOLDING){
            topFalcon.set(ControlMode.Velocity, topTargetVelocity);
            bottomFalcon.set(ControlMode.Velocity, bottomTargetVelocity);
        }
        else if (method == ControlMethod.SPIN_UP){
            topFalcon.set(ControlMode.Velocity, topTargetVelocity);
            bottomFalcon.set(ControlMode.Velocity, bottomTargetVelocity);
        }
    }

    public void configShooterOutputs() {
        topFalcon.configNominalOutputForward(0, 0);
        bottomFalcon.configNominalOutputForward(0, 0);

        topFalcon.configNominalOutputReverse(0, 0);
		bottomFalcon.configNominalOutputReverse(0, 0);
        
        topFalcon.configPeakOutputForward(1, 0);
		bottomFalcon.configPeakOutputForward(1, 0);
        
        topFalcon.configPeakOutputReverse(-1, 0);
        bottomFalcon.configPeakOutputReverse(-1, 0);
    }

    public void setTopPIDF(double kP, double kI, double kD, double kF){
        TalonFuncs.setPIDFConstants(0, topFalcon, kP, kI, kD, kF);
    }

    public void setBottomPIDF(double kP, double kI, double kD, double kF) {
        TalonFuncs.setPIDFConstants(0, bottomFalcon, kP, kI, kD, kF);
    }

    public boolean topOnTarget() {
        return Math.abs(getCurrentTopVelocity() - topTargetVelocity) < velocityThreshold;
    }

    public boolean bottomOnTarget() {
        return Math.abs(getCurrentBottomVelocity() - bottomTargetVelocity) < velocityThreshold;
    }

    public boolean bothOnTarget() {
        return topOnTarget() && bottomOnTarget();
    }

    public void setOutputPower(double power){this.powerOutput = power;}

    public void setTopPower(double power){
        topFalcon.set(ControlMode.PercentOutput, power);
    }
    public void setBottonPower(double power){
        bottomFalcon.set(ControlMode.PercentOutput, power);
    }
    public void setBothPower(double power){
        topFalcon.set(ControlMode.PercentOutput, power);
        bottomFalcon.set(ControlMode.PercentOutput, power);
    }

    public void setkF(double kF){this.kF = kF;}

    public double getTopOutputVoltage(){return topFalcon.getMotorOutputVoltage();}
    public double getBottomOutputVoltage(){return bottomFalcon.getMotorOutputVoltage();}

    public void setControlMethod(ControlMethod method) {
        this.controlMethod = method;
        if(method == ControlMethod.HOLDING){
            setTopPIDF(0,0,0,kF);
            setBottomPIDF(0,0,0,kF);
        }
    }

    public double computekF(double nativeUnits) {
        return 0; // insert regression model
    }

    public double getCurrentTopVelocity(){return topFalcon.getSelectedSensorVelocity(0);}
    public double getCurrentBottomVelocity(){return bottomFalcon.getSelectedSensorVelocity(0);}

    public void setTopTargetVelocity(double velocity){this.topTargetVelocity = velocity;}
    public void setBottomTargetVelocity(double velocity){this.bottomTargetVelocity = velocity;}

    public void percentOutput() {
        topFalcon.set(ControlMode.PercentOutput, powerOutput);
        bottomFalcon.set(ControlMode.PercentOutput, powerOutput);
    }

    public void off() {
        topFalcon.set(ControlMode.PercentOutput, 0);
        bottomFalcon.set(ControlMode.PercentOutput, 0);
    }

    public double getTopPercentOutput() {return topFalcon.getMotorOutputPercent();}
    public double getBottomPercentOutput() {return bottomFalcon.getMotorOutputPercent();}

    // public double getkP(){return kP;}
    // public double getkI(){return kI;}
    // public double getkD(){return kD;}
    public double getkF(){return kF;}

    public TalonFX getTopTalon(){return topFalcon;}
    public TalonFX getBottomTalon(){return bottomFalcon;}

}