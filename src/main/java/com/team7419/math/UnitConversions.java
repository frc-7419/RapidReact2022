package com.team7419.math;

public class UnitConversions {

    public static double rpmToRadPerSec(double rpm){
        return rpm * 2 * Math.PI / 60;
    }

    public static double mpsToRPM(double mps, double radius) { // m is meters
        return (60 * mps)/(2 * Math.PI * radius);
    }

    public static double rpmToMPS(double rpm, double radius) {
        return (2 * Math.PI * radius) / 60;
    }

    public static int mPSToTicksP100Ms(double input) { // m is meters
		double output = (((input*39.3701)/6)*2048)/10;
		return (int) Math.round(output);
	}

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double metersToInches(double meters) { 
        return meters * 39.3701;
    }

    public static double rawSensorVelocityToRPM(double rawVelocity, double ticksPerRotation) {
        return rawVelocity * (1/ticksPerRotation) * 600;
    }

    public static double rpmToRawSensorVelocity(double rpm, double radius, double gearRatioMultiplier, double ticksPerRotation) {
        return rpm * (2 * Math.PI * radius) * (1/ticksToInches(1, radius, gearRatioMultiplier, ticksPerRotation)) * (60/0.1);
    }

    public static double inchesToTicks(double inches, double radius, double gearRatioMultiplier, double ticksPerRotation) {
        return (ticksPerRotation * inches * gearRatioMultiplier)/(2 * Math.PI * radius);
    }

    public static double ticksToInches(double ticks, double radius, double gearRatioMultiplier, double ticksPerRotation) {
        return (2 * Math.PI * radius * ticks)/(ticksPerRotation * gearRatioMultiplier);
    }

    public static double thetaToInches(double theta, double radius) {
        return theta * (Math.PI/180) * radius;
    }
}