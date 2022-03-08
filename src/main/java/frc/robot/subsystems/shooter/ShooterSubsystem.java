package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team7419.TalonFuncs;
import com.team7419.math.UnitConversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX bottomFalcon;
    private TalonFX topFalcon;

    private SimpleMotorFeedforward topFeedforward;
    private SimpleMotorFeedforward bottomFeedforward;

    private double powerOutput = 0;
    private double bottomTargetRawVelocity = 0; // in native velocity
    private double topTargetRawVelocity = 0; 
    private double velocityThreshold = 30; 

    public ShooterSubsystem(){
        bottomFalcon = new TalonFX(CanIds.bottomShooterFalcon.id);
        topFalcon = new TalonFX(CanIds.topShooterFalcon.id);

        topFeedforward = new SimpleMotorFeedforward(RobotConstants.TopShooterKs, RobotConstants.TopShooterKv);
        bottomFeedforward = new SimpleMotorFeedforward(RobotConstants.BottomShooterKs, RobotConstants.BottomShooterKv);

        // topFeedforward = new SimpleMotorFeedforward(RobotConstants.TopShooterKs, RobotConstants.TopShooterKv, RobotConstants.TopShooterKa);
        // bottomFeedforward = new SimpleMotorFeedforward(RobotConstants.BottomShooterKs, RobotConstants.BottomShooterKv, RobotConstants.BottomShooterKa);

        // bottomFalcon.configFactoryDefault();
        // topFalcon.configFactoryDefault();

        bottomFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        topFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        bottomFalcon.setInverted(false);
        topFalcon.setInverted(false);
        
        configShooterOutputs();
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

        SmartDashboard.putNumber("tKf", getTopkF());
        SmartDashboard.putNumber("bKf", getBottomkF());
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

    public void setTopPower(double power) {
        topFalcon.set(ControlMode.PercentOutput, power);
    }
    public void setBottomPower(double power) {
        bottomFalcon.set(ControlMode.PercentOutput, power);
    }
    public void setBothPower(double power) {
        topFalcon.set(ControlMode.PercentOutput, power);
        bottomFalcon.set(ControlMode.PercentOutput, power);
    }

    public double getTopOutputVoltage(){return topFalcon.getMotorOutputVoltage();}
    public double getBottomOutputVoltage(){return bottomFalcon.getMotorOutputVoltage();}

    public double computeTopkF(double velocityMetersPerSecond) {
        return topFeedforward.calculate(velocityMetersPerSecond) / 12;
    }
    public double computeTopkF(double velocityMetersPerSecond, double accelerationMetersPerSecondSquared) {
        return topFeedforward.calculate(velocityMetersPerSecond, accelerationMetersPerSecondSquared);
    }
    public double computeBottomkF(double velocityMetersPerSecond) {
        return topFeedforward.calculate(velocityMetersPerSecond) / 12;
    }
    public double computeBottomkF(double velocityMetersPerSecond, double accelerationMetersPerSecondSquared) {
        return topFeedforward.calculate(velocityMetersPerSecond, accelerationMetersPerSecondSquared);
    }

    public double getCurrentTopRawVelocity(){return topFalcon.getSelectedSensorVelocity(0);}
    public double getCurrentBottomRawVelocity(){return bottomFalcon.getSelectedSensorVelocity(0);}

    public void setTopClosedLoopVelocity(double velocityMetersPerSecond) {
        topFalcon.set(ControlMode.Velocity, velocityMetersPerSecond * );
    }
    public void setBottomClosedLoopVelocity(double velocityMetersPerSecond) {
        this.bottomTargetRawVelocity = velocity;
    }

    public void percentOutput() {
        topFalcon.set(ControlMode.PercentOutput, powerOutput);
        bottomFalcon.set(ControlMode.PercentOutput, powerOutput);
    }

    public void off() {
        setBothPower(0);
    }

    public double getTopPercentOutput() {return topFalcon.getMotorOutputPercent();}
    public double getBottomPercentOutput() {return bottomFalcon.getMotorOutputPercent();}

    public TalonFX getTopTalon(){return topFalcon;}
    public TalonFX getBottomTalon(){return bottomFalcon;}

}