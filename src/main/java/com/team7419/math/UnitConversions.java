package com.team7419.math;

public class UnitConversions {

    public static double rpmToRadPerSec(double rpm){
        return rpm * 2 * Math.PI / 60;
    }

    public static double mPSToRPM(double mps, double radius) { // m is meters
        return (60*mps)/(2*Math.PI*radius);
    }

    public static int mPSToTicksP100Ms(double input) { // m is meters
		double output = (((input*39.3701)/6)*2048)/10;
		return (int) Math.round(output);
	}

    public static double rawSensorVelocityToRPM(double rawVelocity, double radius, double gearRatioMultiplier, double ticksPerRotation) {
        return rawVelocity * ticksToInches(1, radius, gearRatioMultiplier, ticksPerRotation) * (1/(2 * Math.PI * radius)) * 600;
    }

    public static double inchesToTicks(double inches, double radius, double gearRatioMultiplier, double ticksPerRotation) {
        //(ticks per rotation/diameter of wheels)*inches
        return (ticksPerRotation * inches * gearRatioMultiplier)/(2 * Math.PI * radius);
    }

    public static double ticksToInches(double ticks, double radius, double gearRatioMultiplier, double ticksPerRotation) {
        return (2 * Math.PI * radius * ticks)/(ticksPerRotation * gearRatioMultiplier);
    }

    public static double thetaToInches(double theta, double radius) {
        // convert theta to arc length using radius
        return theta * (Math.PI/180) * radius;
    }
}