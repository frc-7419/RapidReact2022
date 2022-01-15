package com.team7419.math;

public class DriveBaseConversions{
    public static double inchesToTicks(double input) {
		double output = 2048*input/(6*3.14159);
		return output;
    }

  }