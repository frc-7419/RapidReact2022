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
        bottomShooterFalcon(10),
        topShooterFalcon(14),
        intakeSpark(15),  
        loaderVictor(16),
        turretSpark(21),
        transferWheelVictor(25),
        rightElevatorFalcon(50),
        leftElevatorFalcon(51), 
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
        public static final double TalonFXTicksPerRotation = 2048;

        public static final double turretRadius = 1.5625; // placeholder value, will change
        public static final double turretGearRatio = 100/12;

        public static final double bottomShooterWheelRadius = 1.5; // placeholder value, will change
        public static final double topShooterWheelRadius = 1.5;

        // top shooter kS, kV, kA
        public static final double TopShooterKs = 0.73569/(2048*6);
        public static final double TopShooterKv = 0.10952/(2048*6);
        public static final double TopShooterKa = 0.0056306/(2048*6);

        // bottom shooter kS, kV, kA
        public static final double BottomShooterKs = 0.57654/(2048*6);
        public static final double BottomShooterKv = 0.10892/(2048*6);
        public static final double BottomShooterKa = 0.0044231/(2048*6);
    }

    public static class PowerConstants {
        public static final double intakeMultiplier = 1.0;
    }

    public static class PIDConstants {
        // straight with motion magic pid gains
        public static final double DriveBaseMotionMagickP = 0.5;
        public static final double DriveBaseMotionMagickI = 0;
        public static final double DriveBaseMotionMagickD = 0;

        /* turn with gyro gain. add your own here */

        // public static double GyrokP180 = 0.001;
        // public static double GyrokI180 = 0.001;
        // public static double GyrokD180 = 0.0001;
        // Following PIDs need to be changed after the actual angles are tuned;
        public static final double GyrokP = 0.00;
        public static final double GyrokI = 0.00;
        public static final double GyrokD = 0.00;

        public static final double GyrokPNegative49 = 0.002;
        public static final double GyrokINegative49 = 0.003;
        public static final double GyrokDNegative49 = 0.0001;

        //Following PIDs are already tuned
        public static final double GyrokP30 = 0.0085;
        public static final double GyrokI30 = 0.00;
        public static final double GyrokD30 = 0.0001;

        public static final double GyrokP80 = 0.0035;
        public static final double GyrokI80 = 0.000655;
        public static final double GyrokD80 = 0.0004;

        public static final double GyrokP115 = 0.005;
        public static final double GyrokI115 = 0.00;
        public static final double GyrokD115 = 0.0000083;
        
        public static final double GyrokP161 = 0.0047;
        public static final double GyrokI161 = 0.00;
        public static final double GyrokD161 = 0.0000083;
        // turret PID gains
        public static final double TurretKp = 0.006;
        public static final double TurretKi = 0;
        public static final double TurretKd = 0;

        //shooter
        public static final double BottomShooterkP = 0.17054/(2048*6);
        public static final double BottomShooterkI = 0;
        public static final double BottomShooterkD = 0;

        public static final double TopShooterkP = 0.10603/(2048*6);
        public static final double TopShooterkI = 0;
        public static final double TopShooterkD = 0;
    }

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
