package frc.robot.subsystems.drive;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.TalonFuncs;
import com.team7419.math.DriveBaseConversions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PowerConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;

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
    public StraightWithMotionMagic(DriveBaseSubsystem driveBase, double setpoint) {
        // this.setpoint = setpoint;
        this.driveBase = driveBase;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize(){

        SmartDashboard.putString("command status", "motion magic test");
        /* factory default just so nothing acts up */
        driveBase.getLeftMast().configFactoryDefault();
        driveBase.getRightMast().configFactoryDefault();
        driveBase.getLeftFollow().configFactoryDefault();
        driveBase.getRightFollow().configFactoryDefault();

        driveBase.getRightMast().setInverted(true);
        driveBase.getRightFollow().setInverted(true);

        driveBase.getLeftMast().setInverted(false);
        driveBase.getLeftFollow().setInverted(false);
        
    //   driveBase.getLeftMast().getSensorCollection().setIntegratedSensorPosition(0, 10);
    //   driveBase.getRightMast().getSensorCollection().setIntegratedSensorPosition(0, 10); 

        driveBase.getLeftMast().setSelectedSensorPosition(0);
        driveBase.getRightMast().setSelectedSensorPosition(0);

        // because sample code 
        driveBase.getLeftMast().configMotionCruiseVelocity(15000, 0);
        driveBase.getLeftMast().configMotionAcceleration(6000, 0);

        driveBase.getRightMast().configMotionCruiseVelocity(15000, 0);
        driveBase.getRightMast().configMotionAcceleration(6000, 0);  

        TalonFuncs.setPIDFConstants(0, driveBase.getLeftMast(), PowerConstants.DriveBaseMotionMagickP.val, 0, PowerConstants.DriveBaseMotionMagickD.val, 0);
        TalonFuncs.setPIDFConstants(0, driveBase.getRightMast(), PowerConstants.DriveBaseMotionMagickP.val, 0, PowerConstants.DriveBaseMotionMagickD.val, 0);
        // setpoint = Dashboard.get(DashboardValue.driveBaseSetpoint);
        double leftSet = DriveBaseConversions.inchesToTicks(setpoint);
        double rightSet = DriveBaseConversions.inchesToTicks(setpoint);

        SmartDashboard.putNumber("leftSet", leftSet);
        SmartDashboard.putNumber("rightSet", rightSet);

        started = false;

        driveBase.getLeftMast().set(ControlMode.MotionMagic, leftSet);
        driveBase.getRightMast().set(ControlMode.MotionMagic, rightSet);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute(){

        SmartDashboard.putString("command status", "executing motion magic");

        SmartDashboard.putNumber("leftMast", driveBase.getLeftMast().getSelectedSensorPosition(0));
        SmartDashboard.putNumber("rightMast", driveBase.getRightMast().getSelectedSensorPosition(0));
    
        double leftMastOutput = driveBase.getLeftMast().getMotorOutputPercent();
        double rightMastOutput = driveBase.getRightMast().getMotorOutputPercent();
        SmartDashboard.putNumber("leftMastOutput", leftMastOutput);
        SmartDashboard.putNumber("rightMastOutput", rightMastOutput);
        SmartDashboard.putNumber("error", driveBase.getLeftMast().getClosedLoopError());
        if(System.currentTimeMillis() - startTime > 1000){
            started = true;
        }

        SmartDashboard.putBoolean("started", started);
        
    }

    @Override
    public boolean isFinished(){
        if(started && Math.abs(leftMastOutput) < 0.01 && Math.abs(rightMastOutput) < 0.01){
            SmartDashboard.putString("command status", "moving");
            Timer.delay(1);
            return true;
        } else{return false;}
    }

    @Override
    public void end(boolean interrupted){

    }
}
