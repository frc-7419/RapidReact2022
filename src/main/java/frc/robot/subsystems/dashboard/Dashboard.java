// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dashboard;

import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Dashboard {

    private DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();

    private ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drive Base");

    private NetworkTableEntry setpoint = driveBaseTab.add("Setpoint", 0).getEntry();

    public Dashboard() {
        SmartDashboard.putData("Straight With Motion Magic", new StraightWithMotionMagic(driveBaseSubsystem, setpoint.getDouble(12)));
    }
    
}
