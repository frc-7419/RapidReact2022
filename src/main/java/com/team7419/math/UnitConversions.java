package com.team7419.math;

public class UnitConversions{

    public static double rpmToRadPerSec(double rpm){
        return rpm * 2 * Math.PI / 60;
    }

    public static double mPSToRPM(double mps, double radius) {
        return (60*mps)/(2*Math.PI*radius);
    }

    public static int mPSToTicksP100Ms(double input){ // m is meters
		double output = (((input*39.3701)/6)*2048)/10;
		return (int) Math.round(output);
	}

    public static double inchesToTicksTalonFX(double inches, double radius){
        //(ticks per rotation/diameter of wheels)*inches
        //delete the diameter parameter after finding out what it is
        return (2048 * inches)/(2 * Math.PI * radius);
    }

    public static double thetaToInches(double theta, double radius) {
        // convert theta to arc length using radius
        return theta * (Math.PI/180) * radius;
    }
    
    public static double thetaToTicks(double theta, double radius) { // theta in degrees, radius in inches
        // convert theta to inches (arc), then convert to ticks
        return inchesToTicksTalonFX(thetaToInches(theta, radius), radius);
    }

    
}