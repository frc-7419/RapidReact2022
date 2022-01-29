// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dashboard;

import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Dashboard extends SubsystemBase {
    
    private DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();

    public static ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drive Base Tab");

    public static NetworkTableEntry motionMagicSetpoint = driveBaseTab.add("Setpoint", 12).getEntry();

    public static NetworkTableEntry motionMagickP = driveBaseTab.add("kP", PIDConstants.DriveBaseMotionMagickP).getEntry();
    public static NetworkTableEntry motionMagickI = driveBaseTab.add("kI", PIDConstants.DriveBaseMotionMagickI).getEntry();
    public static NetworkTableEntry motionMagickD = driveBaseTab.add("kD", PIDConstants.DriveBaseMotionMagickD).getEntry();

    @Override
    public void periodic() {
        SmartDashboard.putData("Straight With Motion Magic", new StraightWithMotionMagic(driveBaseSubsystem, Dashboard.motionMagicSetpoint.getDouble(12)));
    }
    
}
