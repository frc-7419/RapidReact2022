package com.team7419.math;

public class UnitConversions{

    public static double rpmToRadPerSec(double rpm){
        return rpm * 2 * Math.PI / 60;
    }

    public static int mPSToTicksP100Ms(double input){ // m is meters thanks for commenting this neha cough cough
		double output = (((input*39.3701)/6)*2048)/10;
		return (int) Math.round(output);
	}

    public static double degreesToTicks(double degrees, double gearRatioMultiplier,
    double ticksPerRotation) {
        return (ticksPerRotation * gearRatioMultiplier) * (degrees / 360);
    }
}