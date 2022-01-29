package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.TalonFuncs;
import com.team7419.math.DriveBaseConversions;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PowerConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.dashboard.Dashboard;

public class StraightWithMotionMagic extends CommandBase {
  
    private DriveBaseSubsystem driveBaseSubsystem;

    private double setpoint;
    private double leftMastOutput;
    private double rightMastOutput;
    private double leftMastError;
    private double rightMastError;

    private double threshold = 60; // in ticks

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

        // set PIDF constants, using shuffleboard
        TalonFuncs.setPIDFConstants(0, driveBaseSubsystem.getLeftMast(), Dashboard.motionMagickP.getDouble(PIDConstants.DriveBaseMotionMagickP), Dashboard.motionMagickI.getDouble(PIDConstants.DriveBaseMotionMagickI), Dashboard.motionMagickD.getDouble(PIDConstants.DriveBaseMotionMagickD), 0);
        TalonFuncs.setPIDFConstants(0, driveBaseSubsystem.getRightMast(), Dashboard.motionMagickP.getDouble(PIDConstants.DriveBaseMotionMagickP), Dashboard.motionMagickI.getDouble(PIDConstants.DriveBaseMotionMagickI), Dashboard.motionMagickD.getDouble(PIDConstants.DriveBaseMotionMagickD), 0);
        
        // setpoint = Dashboard.get(DashboardValue.driveBaseSetpoint);
        double leftSetpoint = DriveBaseConversions.inchesToTicks(setpoint);
        double rightSetpoint = DriveBaseConversions.inchesToTicks(setpoint);

        SmartDashboard.putNumber("left setpoint", leftSetpoint);
        SmartDashboard.putNumber("right setpoint", rightSetpoint);

        driveBaseSubsystem.getLeftMast().set(ControlMode.MotionMagic, leftSetpoint);
        driveBaseSubsystem.getRightMast().set(ControlMode.MotionMagic, rightSetpoint);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){

        SmartDashboard.putString("command status", "executing motion magic");

        SmartDashboard.putNumber("leftMast", driveBaseSubsystem.getLeftMast().getSelectedSensorPosition(0));
        SmartDashboard.putNumber("rightMast", driveBaseSubsystem.getRightMast().getSelectedSensorPosition(0));
    
        leftMastOutput = driveBaseSubsystem.getLeftMast().getMotorOutputPercent();
        rightMastOutput = driveBaseSubsystem.getRightMast().getMotorOutputPercent();

        leftMastError = driveBaseSubsystem.getLeftMast().getClosedLoopError();
        rightMastError = driveBaseSubsystem.getRightMast().getClosedLoopError();

        SmartDashboard.putNumber("leftMastOutput", leftMastOutput);
        SmartDashboard.putNumber("rightMastOutput", rightMastOutput);
        SmartDashboard.putNumber("left closed loop error", leftMastError);

    }

    @Override
    public boolean isFinished(){
        // threshold: motor output < 0.01
        return (Math.abs(leftMastError) < threshold && Math.abs(rightMastError) < threshold);
    }

    @Override
    public void end(boolean interrupted) {}
}
