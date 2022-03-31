package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team7419.TalonFuncs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIds;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX bottomFalcon;
    private TalonFX topFalcon;
    private double kP;
    private double kI;
    private double tkF;
    private double bkF;
    private double powerOutput = 0;
    private double bottomTargetRawVelocity = 500; // in raw velocity units (ticks/0.1s)
    private double topTargetRawVelocity = 500; // RV
    private double velocityThreshold = 30;  // RV
    private ControlMethod controlMethod = ControlMethod.PERCENT_OUTPUT;

    public ShooterSubsystem(){
        bottomFalcon = new TalonFX(CanIds.bottomShooterFalcon.id);
        topFalcon = new TalonFX(CanIds.topShooterFalcon.id);

        // bottomFalcon.configFactoryDefault();
        // topFalcon.configFactoryDefault();

        bottomFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        topFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        bottomFalcon.setInverted(false);
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
        SmartDashboard.putNumber("tRV", topFalcon.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("bRV", bottomFalcon.getSelectedSensorVelocity(0));

        // SmartDashboard.putNumber("tRPM", UnitConversions.rawSensorVelocityToRPM(topFalcon.getSelectedSensorVelocity(0), 2048));
        // SmartDashboard.putNumber("bRPM", UnitConversions.rawSensorVelocityToRPM(bottomFalcon.getSelectedSensorVelocity(0), 2048));

        // SmartDashboard.putNumber("tRPM Graph", UnitConversions.rawSensorVelocityToRPM(topFalcon.getSelectedSensorVelocity(0), 2048));
        // SmartDashboard.putNumber("bRPM Graph", UnitConversions.rawSensorVelocityToRPM(bottomFalcon.getSelectedSensorVelocity(0), 2048));

        SmartDashboard.putNumber("tError", getCurrentTopRawVelocity() - topTargetRawVelocity);
        SmartDashboard.putNumber("bError", getCurrentBottomRawVelocity() - bottomTargetRawVelocity);
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
        return Math.abs(getCurrentTopRawVelocity() - topTargetRawVelocity) < velocityThreshold;
    }

    public boolean bottomOnTarget() {
        return Math.abs(getCurrentBottomRawVelocity() - bottomTargetRawVelocity) < velocityThreshold;
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

    public void setTopkF(double kF){this.tkF = tkF;}

    public void setBottomkF(double kF){this.bkF = bkF;}

    public double getTopOutputVoltage(){return topFalcon.getMotorOutputVoltage();}
    public double getBottomOutputVoltage(){return bottomFalcon.getMotorOutputVoltage();}

    public void setControlMethod(ControlMethod method) {
        this.controlMethod = method;
        if(method == ControlMethod.HOLDING){
            setTopPIDF(0,0,0,tkF);
            setBottomPIDF(0,0,0,bkF);
        }
    }

    public double computeTopkF(double nativeUnits) {
        return 0; // insert top ff regression model
    }

    public double computeBottomkF(double nativeUnits) {
        return 0; // insert bottom ff regression model
    }

    public double lookUpTopkF(double nativeUnits){
        double output = 0;
        for(double[] pair : Constants.kRawVelocityToTopFf){
            if(pair[0] == nativeUnits){output = pair[1];}
        }
        if(output == 0){output = computeTopkF(nativeUnits);}
        return output; 
    }

    public double lookUpBottomkF(double nativeUnits){
        double output = 0;
        for(double[] pair : Constants.kRawVelocityToBottomFf){
            if(pair[0] == nativeUnits){output = pair[1];}
        }
        if(output == 0){output = computeBottomkF(nativeUnits);}
        return output; 
    }

    public double getCurrentTopRawVelocity(){return topFalcon.getSelectedSensorVelocity(0);}
    public double getCurrentBottomRawVelocity(){return bottomFalcon.getSelectedSensorVelocity(0);}

    public void setTopTargetRawVelocity(double velocity){this.topTargetRawVelocity = velocity;}
    public void setBottomTargetRawVelocity(double velocity){this.bottomTargetRawVelocity = velocity;}

    public void percentOutput() {
        topFalcon.set(ControlMode.PercentOutput, powerOutput);
        bottomFalcon.set(ControlMode.PercentOutput, powerOutput);
    }

    public void off() {
        setBothPower(0);
    }

    public double rpmToRawSensorVelocity(double rpm, double ticksPerRotation) {
        return rpm * ticksPerRotation * (1/600);
    }

    public double getTopPercentOutput() {return topFalcon.getMotorOutputPercent();}
    public double getBottomPercentOutput() {return bottomFalcon.getMotorOutputPercent();}

    public double getkP(){return kP;}
    public double getkI(){return kI;}
    public double getTopkF(){return tkF;}
    public double getBottomkF(){return bkF;}

    public TalonFX getTopTalon(){return topFalcon;}
    public TalonFX getBottomTalon(){return bottomFalcon;}

}