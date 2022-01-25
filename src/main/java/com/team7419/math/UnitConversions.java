package com.team7419.math;

public class UnitConversions{

    public enum MotorController {
        TalonFX,
        TalonSRX,
        SparkMAX,
    }

    public static double rpmToRadPerSec(double rpm) {
        return rpm * 2 * Math.PI / 60;
    }

    public static double mPSToRPM(double mps, double radius) {
        return (60*mps)/(2*Math.PI*radius);
    }

    public static int mPSToTicksP100Ms(double input) { // m is meters
		double output = (((input*39.3701)/6)*2048)/10;
		return (int) Math.round(output);
	}

    public static double inchesToTicks(MotorController motorController, double inches, double radius) {
        //(ticks per rotation/diameter of wheels)*inches
        if (motorController == MotorController.TalonFX) {
            return (2048 * inches)/(2 * Math.PI * radius);
        }
        else if (motorController == MotorController.TalonSRX || motorController == MotorController.SparkMAX) {
            return (4096 * inches)/(2 * Math.PI * radius);
        }
        else {
           return 0;
        }
    }

    public static double thetaToInches(double theta, double radius) {
        // convert theta to arc length using radius
        return theta * (Math.PI/180) * radius;
    }
    
}