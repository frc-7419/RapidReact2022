package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.TalonFuncs;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;

public class StraightWithMotionMagic extends CommandBase {
  
    private DriveBaseSubsystem driveBaseSubsystem;

    private double setpoint;
    private double leftMastOutput;
    private double rightMastOutput;
    private double leftMastError;
    private double rightMastError;

    private double threshold = 0.5; // in inches

    private double kP;
    private double kI;
    private double kD;

    private long startTime;
   
    public StraightWithMotionMagic(DriveBaseSubsystem driveBaseSubsystem, double setpoint) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.setpoint = setpoint;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("command status", "motion magic test");
        /* factory default just so nothing acts up */
        // driveBaseSubsystem.factoryResetAll();

        // reset default inversions
        driveBaseSubsystem.setAllDefaultInversions();

        // reset sensor position
        driveBaseSubsystem.getLeftMast().setSelectedSensorPosition(0);
        driveBaseSubsystem.getRightMast().setSelectedSensorPosition(0);

        // sample code
        driveBaseSubsystem.getLeftMast().configMotionCruiseVelocity(15000, 0);
        driveBaseSubsystem.getLeftMast().configMotionAcceleration(6000, 0);

        driveBaseSubsystem.getRightMast().configMotionCruiseVelocity(15000, 0);
        driveBaseSubsystem.getRightMast().configMotionAcceleration(6000, 0);  

        // PIDF gains from SmartDashboard
        kP = SmartDashboard.getNumber("mmKp", PIDConstants.DriveBaseMotionMagickP);
        kD = SmartDashboard.getNumber("mmKi", PIDConstants.DriveBaseMotionMagickI);
        kI = SmartDashboard.getNumber("mmKd", PIDConstants.DriveBaseMotionMagickD);

        // set PIDF constants
        TalonFuncs.setPIDFConstants(0, driveBaseSubsystem.getLeftMast(), kP, kD, kI, 0);
        TalonFuncs.setPIDFConstants(0, driveBaseSubsystem.getRightMast(), kP, kD, kI, 0);
        
        // get setpoint from SmartDashboard
        setpoint = SmartDashboard.getNumber("mmSetpoint", 12);
        double leftSetpoint = UnitConversions.inchesToTicks(setpoint);
        double rightSetpoint = UnitConversions.inchesToTicks(setpoint);

        SmartDashboard.putNumber("left setpoint", leftSetpoint);
        SmartDashboard.putNumber("right setpoint", rightSetpoint);

        driveBaseSubsystem.getLeftMast().set(ControlMode.MotionMagic, leftSetpoint);
        driveBaseSubsystem.getRightMast().set(ControlMode.MotionMagic, rightSetpoint);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){

        SmartDashboard.putString("command status", "executing motion magic");

        SmartDashboard.putNumber("leftMast position", driveBaseSubsystem.getLeftMast().getSelectedSensorPosition(0));
        SmartDashboard.putNumber("rightMast position", driveBaseSubsystem.getRightMast().getSelectedSensorPosition(0));
    
        leftMastOutput = driveBaseSubsystem.getLeftMast().getMotorOutputPercent();
        rightMastOutput = driveBaseSubsystem.getRightMast().getMotorOutputPercent();

        leftMastError = driveBaseSubsystem.getLeftMast().getClosedLoopError();
        rightMastError = driveBaseSubsystem.getRightMast().getClosedLoopError();

        SmartDashboard.putNumber("leftMastOutput", leftMastOutput);
        SmartDashboard.putNumber("rightMastOutput", rightMastOutput);
        SmartDashboard.putNumber("left closed loop error", leftMastError);
        SmartDashboard.putNumber("right closed loop error", rightMastError);
    }

    @Override
    public boolean isFinished(){
        double thresholdInTicks = UnitConversions.inchesToTicks(threshold);
        return (Math.abs(leftMastError) < thresholdInTicks && Math.abs(rightMastError) < thresholdInTicks);
    }

    @Override
    public void end(boolean interrupted) {}
}
