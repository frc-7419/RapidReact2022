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
    private double powerOutput = 0;
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double bottomTargetRawVelocity = 500;
    private double topTargetRawVelocity = 500;
    private double velocityThreshold = 200;
    private ControlMethod controlMethod = ControlMethod.PERCENT_OUTPUT;

    public ShooterSubsystem(){
        bottomFalcon = new TalonFX(CanIds.topShooterFalcon.id);
        topFalcon = new TalonFX(CanIds.bottomShooterFalcon.id);

        bottomFalcon.configFactoryDefault();
        topFalcon.configFactoryDefault();

        bottomFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        topFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        bottomFalcon.setInverted(true);
        topFalcon.setInverted(false);
        
        // configureShooterOutputs();
    }

    public enum ControlMethod {
        PERCENT_OUTPUT,
        SPIN_UP,
        HOLDING,
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("top falcon raw speed", topFalcon.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("bottom falcon raw speed", bottomFalcon.getSelectedSensorVelocity(0));

        SmartDashboard.putNumber("top falcon rpm", UnitConversions.rawSensorVelocityToRPM(topFalcon.getSelectedSensorVelocity(0), 2048));
        SmartDashboard.putNumber("bottom falcon rpm", UnitConversions.rawSensorVelocityToRPM(bottomFalcon.getSelectedSensorVelocity(0), 2048));
    }

    public void run() {
        ControlMethod method = this.controlMethod;
        if (method == ControlMethod.PERCENT_OUTPUT){
            topFalcon.set(ControlMode.PercentOutput, powerOutput);
            bottomFalcon.set(ControlMode.PercentOutput, powerOutput);
        }
        else if (method == ControlMethod.HOLDING){
            topFalcon.set(ControlMode.Velocity, topTargetRawVelocity);
            bottomFalcon.set(ControlMode.Velocity, bottomTargetRawVelocity);
        }
        else if (method == ControlMethod.SPIN_UP){
            topFalcon.set(ControlMode.Velocity, topTargetRawVelocity);
            bottomFalcon.set(ControlMode.Velocity, bottomTargetRawVelocity);
        }
    }

    public void configureShooterOutputs() {
        topFalcon.configNominalOutputForward(0, 0);
        bottomFalcon.configNominalOutputForward(0, 0);

        topFalcon.configNominalOutputReverse(0, 0);
		bottomFalcon.configNominalOutputReverse(0, 0);
        
        topFalcon.configPeakOutputForward(1, 0);
		bottomFalcon.configPeakOutputForward(1, 0);
        
        topFalcon.configPeakOutputReverse(-1, 0);
        bottomFalcon.configPeakOutputReverse(-1, 0);
    }

    public void setPIDF(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        TalonFuncs.setPIDFConstants(0, topFalcon, kP, kI, kD, kF);
        TalonFuncs.setPIDFConstants(0, bottomFalcon, kP, kI, kD, kF);
    }

    public boolean topOnTarget() {
        return Math.abs(getCurrentTopVelocity() - topTargetRawVelocity) < velocityThreshold;
    }

    public boolean bottomOnTarget() {
        return Math.abs(getCurrentBottomVelocity() - bottomTargetRawVelocity) < velocityThreshold;
    }

    public boolean bothOnTarget() {
        return topOnTarget() && bottomOnTarget();
    }

    public void setOutputPower(double power){this.powerOutput = power;}
    
    public void setTopPower(double power){
        topFalcon.set(ControlMode.PercentOutput, power);
    }
    public void setBottomPower(double power){
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
            setPIDF(0,0,0,kF);
        }
    }

    public double computekF(double nativeUnits) {
        return 0; // insert regression model
    }

    public double getCurrentTopVelocity(){return topFalcon.getSelectedSensorVelocity(0);}
    public double getCurrentBottomVelocity(){return bottomFalcon.getSelectedSensorVelocity(0);}

    public void setTopTargetRawVelocity(double velocity){this.topTargetRawVelocity = velocity;}
    public void setBottomTargetRawVelocity(double velocity){this.bottomTargetRawVelocity = velocity;}

    public void percentOutput() {
        topFalcon.set(ControlMode.PercentOutput, powerOutput);
        bottomFalcon.set(ControlMode.PercentOutput, powerOutput);
    }

    public void off() {
        topFalcon.set(ControlMode.PercentOutput, 0);
        bottomFalcon.set(ControlMode.PercentOutput, 0);
    }

    public double getkP(){return kP;}
    public double getkI(){return kI;}
    public double getkD(){return kD;}
    public double getkF(){return kF;}

    public TalonFX getTopTalon(){return topFalcon;}
    public TalonFX getBottomTalon(){return bottomFalcon;}

}