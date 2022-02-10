// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // enums are a data structure primarilly used for constants
    public static enum CanIds {
        
        //to add a motor's can id: motorName(canId),
        // motor1(5),
        // motor2(3),
        // motorX(x)...

        // 2020 robot constants
        // Drivetrain Constants 
        leftFalcon1(5),
        rightFalcon1(2),
        leftFalcon2(4),
        rightFalcon2(3),

        loaderFalcon(10),
        intakeVictor(11),
        revolverVictor(12),
        climberFalcon(13),
        shooterFalcon(14), 
        hoodVictor(40), 
        ;

        public final int id;
        private CanIds(int id) {
            this.id = id;
        }
    }

    public static class LimelightConstants {
        public static double kTargetHeight = 2.6416; //meters
        public static double kCameraHeight = 0.8128;
        public static double mountingAngle = 55; 
        public static double m = 1.0; 
        public static double n = 0.48; //arbitrary
        public static double r1 = 0.37; //arbitrary
        public static double g = -9.8;  
        public static double focalLength = 2.9272781257541;
    }

    // values for Gains Constants fron System Identification Tool 
    public static class TrajectoryConstants {
        
        // Constants change depending on the Units 
        // Double check the constant depening on the units. can change on SysId tool
        public static final double ks = 0.0; // Units: Volts // change me 
        public static final double kv = 0; // Units: VoltSecondsPerMeter // change me 
        public static final double ka = 0; // Units: VoltSecondsSquaredPerMeter // change me 

        public static final double kPDriveVel = 0; // change me 
        
        public static final double kTrackWidth = 0; // Units: Meters // change me 
        public static final double kMaxSpeed = 0; // Units: MetersPerSecond //change me 
        public static final double kMaxAcceleration = 0; // Units: MetersPerSecondSquared // change me

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
    }

    public static class RobotConstants {
        public static double shooterRadius = 0.5; // placeholder value, will change
    }

    public static class PIDConstants {
        
    }

    public static double[][] kSpeedToFf = {
        {1000, .09},
        {2000, .065},
        {3000, .0575},
        {4000, .0545},
        {5000, .0525},
        {6000, .051},
        {7000, .0505},
        {8000, .0495},
        {9000, .049},
        {10000, .0485},
        {11000, .0482},
        {12000, .0478},
        {13000, .0475},
        {14000, .0474},
        {15000, .0474},
        {16000, .0474},
        {17000, .0472},
        {18000, .047},
        {19000, .047},
        {20000, .047},
        {21000, .0472},
        {22000, .0472},
    };           
}
