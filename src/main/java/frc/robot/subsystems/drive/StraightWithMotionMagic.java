/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team7419.TalonFuncs;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;

public class StraightWithMotionMagic extends CommandBase {
  
    private DriveBaseSubsystem driveBase;
    private double setpoint;
    private double leftMastOutput;
    private double rightMastOutput;
    private boolean started;
    private long startTime;

    /**
     * 
     * @param driveBase
     * @param setpoint in inches
     */
    public StraightWithMotionMagic(DriveBaseSubsystem driveBaseSubsystem, double setpoint) {
        // this.setpoint = setpoint;
        this.driveBase = driveBaseSubsystem;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize(){
        SmartDashboard.putBoolean("MM Running", false);

        /* factory default + inversions just so nothing acts up */
        driveBase.factoryResetAll();
        driveBase.setAllDefaultInversions();
        driveBase.setAllMode(NeutralMode.Coast);

        driveBase.getLeftMast().setSelectedSensorPosition(0);
        driveBase.getRightMast().setSelectedSensorPosition(0);

        // because sample code 
        driveBase.getLeftMast().configMotionCruiseVelocity(15000, 0);
        driveBase.getLeftMast().configMotionAcceleration(6000, 0);

        driveBase.getRightMast().configMotionCruiseVelocity(15000, 0);
        driveBase.getRightMast().configMotionAcceleration(6000, 0);  

        TalonFuncs.setPIDFConstants(0, driveBase.getLeftMast(), PIDConstants.DriveBaseMotionMagickP, PIDConstants.DriveBaseMotionMagickI, PIDConstants.DriveBaseMotionMagickD, 0);
        TalonFuncs.setPIDFConstants(0, driveBase.getRightMast(), PIDConstants.DriveBaseMotionMagickP, PIDConstants.DriveBaseMotionMagickI, PIDConstants.DriveBaseMotionMagickD, 0);

        double leftSetpoint = UnitConversions.inchesToTicks(setpoint, 3, 10.71, 2048);
        double rightSetpoint = UnitConversions.inchesToTicks(setpoint, 3, 10.71, 2048);

        SmartDashboard.putNumber("lSetpoint", leftSetpoint);
        SmartDashboard.putNumber("rSetpoint", rightSetpoint);

        started = false;

        driveBase.getLeftMast().set(ControlMode.MotionMagic, leftSetpoint);
        driveBase.getRightMast().set(ControlMode.MotionMagic, rightSetpoint);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){

        SmartDashboard.putBoolean("MM Running", true);

        SmartDashboard.putNumber("LM Position", driveBase.getLeftMast().getSelectedSensorPosition(0));
        SmartDashboard.putNumber("RM Position", driveBase.getRightMast().getSelectedSensorPosition(0));
    
        double leftMastOutput = driveBase.getLeftMast().getMotorOutputPercent();
        double rightMastOutput = driveBase.getRightMast().getMotorOutputPercent();
        SmartDashboard.putNumber("LM Out", leftMastOutput);
        SmartDashboard.putNumber("RM Out", rightMastOutput);
        SmartDashboard.putNumber("LM Error", driveBase.getLeftMast().getClosedLoopError());
        SmartDashboard.putNumber("RM Error", driveBase.getRightMast().getClosedLoopError());
        if(System.currentTimeMillis() - startTime > 1000){
            started = true;
        }

        SmartDashboard.putBoolean("started", started);
    }

    @Override
    public boolean isFinished(){
        if(started && Math.abs(leftMastOutput) < 0.01 && Math.abs(rightMastOutput) < 0.01){
            Timer.delay(1);
            return true;
        } else{return false;}
    }

    @Override
    public void end(boolean interrupted){
        driveBase.stop();
        driveBase.brake();
        SmartDashboard.putBoolean("MM Running", false);
    }
}
