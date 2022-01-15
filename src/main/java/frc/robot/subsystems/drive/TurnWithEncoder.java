/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.TalonFuncs;
import com.team7419.math.DriveBaseConversions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PowerConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem.TurnDirection;

public class TurnWithEncoder extends CommandBase {
  
    private DriveBaseSubsystem driveBase;
    private double setpoint;
    private double leftMastOutput;
    private double rightMastOutput;
    private boolean started;
    private long startTime;
    private boolean setRightInverted;
    private TurnDirection turnDirection;

    /**
     * 
     * @param driveBase
     * @param setpoint in inches
     */
    public TurnWithEncoder(DriveBaseSubsystem driveBase, double setpoint, TurnDirection turnDirection) { 
        // if setRightInverted, then the right motors are inverted, else the left motors are inverted

        // this.setpoint = setpoint;
        this.driveBase = driveBase;
        this.setpoint = setpoint;
        this.turnDirection = turnDirection;
        // this.setRightInverted = setRightInverted;
    }

    @Override
    public void initialize(){
        if (turnDirection == TurnDirection.RIGHT) {
            // for right turn set only one as inverted
            driveBase.getRightMast().setInverted(false);
            driveBase.getRightFollow().setInverted(false);
        }
        else {
            // for left turn set both as inverted
            driveBase.getRightMast().setInverted(true);
            driveBase.getRightFollow().setInverted(true);

            driveBase.getLeftMast().setInverted(true);
            driveBase.getLeftFollow().setInverted(true);
        }

        SmartDashboard.putString("command status", "motion magic test");
        /* factory default just so nothing acts up */
        // driveBase.getRightMast().configFactoryDefault();
    //   driveBase.getLeftMast().configFactoryDefault();

    //   driveBase.getLeftMast().getSensorCollection().setIntegratedSensorPosition(0, 10);
    //   driveBase.getRightMast().getSensorCollection().setIntegratedSensorPosition(0, 10); 

        driveBase.getLeftMast().setSelectedSensorPosition(0);
        driveBase.getRightMast().setSelectedSensorPosition(0);

        // because sample code 
        driveBase.getLeftMast().configMotionCruiseVelocity(15000, 0);
        driveBase.getLeftMast().configMotionAcceleration(6000, 0);

        driveBase.getRightMast().configMotionCruiseVelocity(15000, 0);
        driveBase.getRightMast().configMotionAcceleration(6000, 0);  

        TalonFuncs.setPIDFConstants(0, driveBase.getLeftMast(), PowerConstants.TurnWithEncoderkP.val, PowerConstants.TurnWithEncoderkI.val, PowerConstants.TurnWithEncoderkD.val, 0);
        TalonFuncs.setPIDFConstants(0, driveBase.getRightMast(), PowerConstants.TurnWithEncoderkP.val, PowerConstants.TurnWithEncoderkI.val, PowerConstants.TurnWithEncoderkD.val, 0);
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
        if(started && Math.abs(leftMastOutput) < PowerConstants.TurnWithEncoderTolerance.val && Math.abs(rightMastOutput) < PowerConstants.TurnWithEncoderTolerance.val){
            SmartDashboard.putString("command status", "moving");
            Timer.delay(1);
            return true;
        } else{return false;}
    }

    @Override
    public void end(boolean interrupted){
        // reset inversions to default
        // driveBase.getRightMast().setInverted(true);
        // driveBase.getRightFollow().setInverted(true);

        // driveBase.getLeftMast().setInverted(false);
        // driveBase.getLeftFollow().setInverted(false);
    }
}
