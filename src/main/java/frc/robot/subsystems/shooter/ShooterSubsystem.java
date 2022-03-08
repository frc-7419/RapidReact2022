package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.team7419.TalonFuncs;
import com.team7419.math.UnitConversions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX bottomFalcon;
    private TalonFX topFalcon;

    private SimpleMotorFeedforward topFeedforward;
    private SimpleMotorFeedforward bottomFeedforward;

    private double bottomTargetVelocity = 0; // MPS
    private double topTargetVelocity = 0; 
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
        // SmartDashboard.putNumber("tRV", topFalcon.getSelectedSensorVelocity(0));
        // SmartDashboard.putNumber("bRV", bottomFalcon.getSelectedSensorVelocity(0));

        SmartDashboard.putNumber("tV", getCurrentTopVelocity());
        SmartDashboard.putNumber("bV", getCurrentBottomVelocity());

        SmartDashboard.putNumber("tError", getCurrentTopRawVelocity() - topTargetVelocity);
        SmartDashboard.putNumber("bError", getCurrentBottomRawVelocity() - bottomTargetVelocity);
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

    public void setTopClosedLoopVelocity(double velocityMetersPerSecond) {
        this.topTargetVelocity = velocityMetersPerSecond;
        topFalcon.set(ControlMode.Velocity, velocityMetersPerSecond * RobotConstants.RotationsPerMeter * 2048 * 0.1, DemandType.ArbitraryFeedForward, topFeedforward.calculate(velocityMetersPerSecond) / RobotController.getBatteryVoltage());
    }
    public void setBottomClosedLoopVelocity(double velocityMetersPerSecond) {
        this.bottomTargetVelocity = velocityMetersPerSecond;
        bottomFalcon.set(ControlMode.Velocity, velocityMetersPerSecond * RobotConstants.RotationsPerMeter * 2048 * 0.1, DemandType.ArbitraryFeedForward, bottomFeedforward.calculate(velocityMetersPerSecond) / RobotController.getBatteryVoltage());
    }

    public void setTopPIDF(double kP, double kI, double kD, double kF){
        TalonFuncs.setPIDFConstants(0, topFalcon, kP, kI, kD, kF);
    }

    public void setBottomPIDF(double kP, double kI, double kD, double kF) {
        TalonFuncs.setPIDFConstants(0, bottomFalcon, kP, kI, kD, kF);
    }

    public boolean topOnTarget() {
        return Math.abs(getCurrentTopRawVelocity() - topTargetVelocity) < velocityThreshold;
    }

    public boolean bottomOnTarget() {
        return Math.abs(getCurrentBottomRawVelocity() - bottomTargetVelocity) < velocityThreshold;
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
    public void off() {
        setBothPower(0);
    }

    public double getTopOutputVoltage(){return topFalcon.getMotorOutputVoltage();}
    public double getBottomOutputVoltage(){return bottomFalcon.getMotorOutputVoltage();}

    // public double computeTopkF(double velocityMetersPerSecond, double accelerationMetersPerSecondSquared) {
    //     return topFeedforward.calculate(velocityMetersPerSecond, accelerationMetersPerSecondSquared) / RobotController.getBatteryVoltage();
    // }
    // public double computeBottomkF(double velocityMetersPerSecond, double accelerationMetersPerSecondSquared) {
    //     return topFeedforward.calculate(velocityMetersPerSecond, accelerationMetersPerSecondSquared) / RobotController.getBatteryVoltage();
    // }

    public double getCurrentTopRawVelocity(){return topFalcon.getSelectedSensorVelocity(0);}
    public double getCurrentBottomRawVelocity(){return bottomFalcon.getSelectedSensorVelocity(0);}

    public double getCurrentTopVelocity(){return UnitConversions.rawSensorVelocityToMPS(getCurrentTopRawVelocity(), 2048, RobotConstants.TopShooterWheelRadius);}
    public double getCurrentBottomVelocity(){return UnitConversions.rawSensorVelocityToMPS(getCurrentBottomRawVelocity(), 2048, RobotConstants.BottomShooterWheelRadius);}

    public double getTopPercentOutput() {return topFalcon.getMotorOutputPercent();}
    public double getBottomPercentOutput() {return bottomFalcon.getMotorOutputPercent();}

    public TalonFX getTopTalon(){return topFalcon;}
    public TalonFX getBottomTalon(){return bottomFalcon;}

}