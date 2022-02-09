package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team7419.TalonFuncs;
import com.team7419.math.UnitConversions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;

public class StraightWithMotionMagicNew extends CommandBase {
  
    private DriveBaseSubsystem driveBaseSubsystem;

    private double setpoint;
    private double leftMastOutput;
    private double rightMastOutput;
    private double leftMastError;
    private double rightMastError;

    private double threshold = 0.5; // in inches

    private int kErrThreshold = 30; // how many sensor units until its close-enough
    private int kLoopsToSettle = 10; // how many loops sensor must be close-enough
    private int withinThresholdLoops = 0;

    private double kP;
    private double kI;
    private double kD;

    private long startTime;
   
    public StraightWithMotionMagicNew(DriveBaseSubsystem driveBaseSubsystem, double setpoint) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.setpoint = setpoint;
        addRequirements(driveBaseSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MM Running", false);

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
        double leftSetpoint = UnitConversions.inchesToTicks(setpoint, 3, 1, 2048);
        double rightSetpoint = UnitConversions.inchesToTicks(setpoint, 3, 1, 2048);

        SmartDashboard.putNumber("lSetpoint", leftSetpoint);
        SmartDashboard.putNumber("rSetpoint", rightSetpoint);

        driveBaseSubsystem.getLeftMast().set(ControlMode.MotionMagic, leftSetpoint);
        driveBaseSubsystem.getRightMast().set(ControlMode.MotionMagic, rightSetpoint);

        startTime = System.currentTimeMillis();

            /* Check if closed loop error is within the threshld */
        if ((driveBaseSubsystem.getRightMast().getClosedLoopError() < kErrThreshold && driveBaseSubsystem.getLeftMast().getClosedLoopError() < kErrThreshold) 
            && (driveBaseSubsystem.getRightMast().getClosedLoopError() > -kErrThreshold && driveBaseSubsystem.getLeftMast().getClosedLoopError() > -kErrThreshold)) {
            ++withinThresholdLoops;
        } else {
            withinThresholdLoops = 0;
        }
    }

    @Override
    public void execute(){

        SmartDashboard.putBoolean("MM Running", true);

        SmartDashboard.putNumber("LM Position", driveBaseSubsystem.getLeftMast().getSelectedSensorPosition(0));
        SmartDashboard.putNumber("RM Position", driveBaseSubsystem.getRightMast().getSelectedSensorPosition(0));

        SmartDashboard.putNumber("LM Position G", driveBaseSubsystem.getLeftMast().getSelectedSensorPosition(0));
        SmartDashboard.putNumber("RM Position G", driveBaseSubsystem.getRightMast().getSelectedSensorPosition(0));
    
        leftMastOutput = driveBaseSubsystem.getLeftMast().getMotorOutputPercent();
        rightMastOutput = driveBaseSubsystem.getRightMast().getMotorOutputPercent();

        leftMastError = driveBaseSubsystem.getLeftMast().getClosedLoopError();
        rightMastError = driveBaseSubsystem.getRightMast().getClosedLoopError();

        SmartDashboard.putNumber("LM pOutput", leftMastOutput);
        SmartDashboard.putNumber("RM pOutput", rightMastOutput);
        SmartDashboard.putNumber("lCL Error", leftMastError);
        SmartDashboard.putNumber("rCL Error", rightMastError);
    }

    @Override
    public boolean isFinished() {

        double rightActiveTrajectoryPosition = driveBaseSubsystem.getRightMast().getActiveTrajectoryPosition();
        double leftActiveTrajectoryPosition = driveBaseSubsystem.getLeftMast().getActiveTrajectoryPosition();
        return (withinThresholdLoops > kLoopsToSettle);
        
            // && (UnitConversions.ticksToInches(rightActiveTrajectoryPosition, 3, 1, 2048) < (setpoint + threshold)) 
            // && (UnitConversions.ticksToInches(rightActiveTrajectoryPosition, 3, 1, 2048) > (setpoint - threshold))

            // && (UnitConversions.ticksToInches(leftActiveTrajectoryPosition, 3, 1, 2048) < (setpoint + threshold)) 
            // && (UnitConversions.ticksToInches(leftActiveTrajectoryPosition, 3, 1, 2048) > (setpoint - threshold));
    }

    @Override
    public void end(boolean interrupted) {
        driveBaseSubsystem.setAll(0);
        driveBaseSubsystem.brake();
        SmartDashboard.putBoolean("MM Running", false);
    }
}
