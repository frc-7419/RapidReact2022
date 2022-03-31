package frc.robot;

public final class Constants {

    public static enum CanIds {
        leftFalcon1(5),
        rightFalcon1(2),
        leftFalcon2(4),
        rightFalcon2(3),
        topShooterFalcon(14), 
        bottomShooterFalcon(10),
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
        public static final double focalLength = 2.9272781257541;
    }

    public static class RobotConstants {
        public static final double TalonFXTicksPerRotation = 2048;
        public static final double bottomShooterWheelRadius = 1.5; // placeholder value, will change
        public static final double topShooterWheelRadius = 1.5;
    }

    public static class PIDConstants {
        //shooter
        public static final double ShooterTopKp = 0;
        public static final double ShooterBottomKp = 0;

        public static final double ShooterTopKi = 0;
        public static final double ShooterBottomKi = 0;
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
