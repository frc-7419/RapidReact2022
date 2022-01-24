package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team7419.TalonFuncs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class ShooterSubsystem extends SubsystemBase{

    private TalonFX shooter;
    private LimelightSubsystem limelight = new LimelightSubsystem();
    private double powerOutput = 0;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;
    private double targetVelocity = 0;
    private double target = 500;
    private double threshold = 200;
    private ControlMethod controlMethod = ControlMethod.PERCENT_OUTPUT;

    private double v0 = Math.sqrt(LimelightConstants.g/(2*limelight.getA()*(Math.pow(Math.cos(Math.toRadians(limelight.getBeta())),2))));

    public ShooterSubsystem(){
        shooter = new TalonFX(CanIds.shooterFalcon.id);
        shooter.configFactoryDefault();
        shooter.setInverted(true);
        shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    }

    public enum ControlMethod{
        PERCENT_OUTPUT,
        SPIN_UP,
        HOLDING,
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("periodic speed", shooter.getSelectedSensorVelocity());
    }

    public void run(){
        ControlMethod method = this.controlMethod;
        if(method == ControlMethod.PERCENT_OUTPUT){
            shooter.set(ControlMode.PercentOutput, powerOutput);
        }
        else if(method == ControlMethod.HOLDING){
            shooter.set(ControlMode.Velocity, target);
        }
        else if(method == ControlMethod.SPIN_UP){
            shooter.set(ControlMode.Velocity, target);
        }

    }

    public void configureOutputs(){
        shooter.configNominalOutputForward(0, 0);
		shooter.configNominalOutputReverse(0, 0);
		shooter.configPeakOutputForward(1, 0);
        shooter.configPeakOutputReverse(-1, 0);
    }

    public void setPIDF(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        TalonFuncs.setPIDFConstants(0, shooter, kP, kI, kD, kF);
    }

    public boolean onTarget(){
        return Math.abs(this.getCurrentRawSpeed() - target) < threshold;
    }

    public void setOutputPower(double power){this.powerOutput = power;}

    public void setkF(double kF){this.kF = kF;}

    public double getOutputVoltage(){
        return shooter.getMotorOutputVoltage();
    }

    public void setTargetRpm(double rpm){this.target = rpm * 1.7067;}

    public void setControlMethod(ControlMethod method){
        this.controlMethod = method;
        if(method == ControlMethod.HOLDING){
            setPIDF(0,0,0,kF);
        }
    }

    public double computekF(double nativeUnits){
        return 0; // insert regression model
    }

    public double getCurrentRawSpeed(){return shooter.getSelectedSensorVelocity(0);}

    public void setTargetRawSpeed(double speed){this.target = speed;}

    public void percentOutput(){
        shooter.set(ControlMode.PercentOutput,powerOutput);
    }

    public void off(){
        shooter.set(ControlMode.PercentOutput, 0);
    }

    public double getkP(){return kP;}
    public double getkI(){return kI;}
    public double getkD(){return kD;}
    public double getkF(){return kF;}

    public double getV0() {return v0;}

    public TalonFX getShooterTalon() {return shooter;}

}