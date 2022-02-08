// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static enum CanIds {

        // 2020 robot constants
        leftFalcon1(5),
        rightFalcon1(2),
        leftFalcon2(4),
        rightFalcon2(3),
        intakeVictor(11),
        topShooterFalcon(13), 
        bottomShooterFalcon(10),
        hoodVictor(40), 
        ;

        public final int id;
        private CanIds(int id) {
            this.id = id;
        }
    }

    public static class LimelightConstants {
        public static final double kTargetHeight = 2.6416; //meters
        public static final double kCameraHeight = 0.8128;
        public static final double mountingAngle = 55; 
        public static final double m = 1.0; 
        public static final double n = 0.48; //arbitrary
        public static final double r1 = 0.37; //arbitrary
        public static final double g = -9.8;  
        public static final double focalLength = 2.9272781257541;
    }

    public static class RobotConstants {
        public static final double bottomShooterWheelRadius = 1.5; // placeholder value, will change
        public static final double topShooterWheelRadius = 1.5;
    }

    public static class PIDConstants {
        //gyro
        public static final double GyrokP = 0.085;
        public static final double GyrokI = 0;
        public static final double GyrokD = 0;

        //shooter
        public static final double ShooterkP = 0;
        public static final double ShooterkI = 0;
        public static final double ShooterkD = 0;
        public static final double ShooterkF = 0.05;
    }

    // old 

    // public static final double[][] kSpeedToFf = {
    //     {1000, .09},
    //     {2000, .065},
    //     {3000, .0575},
    //     {4000, .0545},
    //     {5000, .0525},
    //     {6000, .051},
    //     {7000, .0505},
    //     {8000, .0495},
    //     {9000, .049},
    //     {10000, .0485},
    //     {11000, .0482},
    //     {12000, .0478},
    //     {13000, .0475},
    //     {14000, .0474},
    //     {15000, .0474},
    //     {16000, .0474},
    //     {17000, .0472},
    //     {18000, .047},
    //     {19000, .047},
    //     {20000, .047},
    //     {21000, .0472},
    //     {22000, .0472},
    // };   
    
    public static final double[][] kRawVelocityToTopFf = {
        {1000, 0.06365},
        {1500, 0.058615},
        {2000, 0.0555},
        {2500, 0.0535},
        {3000, 0.0525},
        {3500, 0.0506},
        {4000, 0.0495},
        {4500, 0.049425},
        {5000, 0.0488},
        {5500, 0.0491},
        {6000, 0.048305},
        {6500, 0.048305},
        {7000, 0.0486},
        {7500, 0.048875},
        {8000, 0.04874},
        {8500, 0.04876}
    };

    public static final double[][] kRawVelocityToBottomFf = {
        {1000, 0.0615},
        {1500, 0.0552},
        {2000, 0.05},
        {2500, 0.04815},
        {3000, 0.04775},
        {3500, 0.04735},
        {4000, 0.047545},
        {4500, 0.0473445},
        {5000, 0.0471},
        {5500, 0.0475},
        {6000, 0.047385},
        {6500, 0.0477425},
        {7000, 0.0477425},
        {7500, 0.047761},
        {8000, 0.04772},
        {8500, 0.047827}
    };
}
