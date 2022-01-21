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
}