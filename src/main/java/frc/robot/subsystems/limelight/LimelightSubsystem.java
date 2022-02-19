// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  // set up a new instance of NetworkTables (the api/library used to read values from limelight)
  NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
  
  // return network table values for tx and ty using getEntry()
  NetworkTableEntry tv = networkTable.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1)
  NetworkTableEntry tx = networkTable.getEntry("tx"); // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
  NetworkTableEntry ty = networkTable.getEntry("ty"); // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  NetworkTableEntry ta = networkTable.getEntry("ta"); // Target Area (0% of image to 100% of image)
  NetworkTableEntry ts = networkTable.getEntry("ts"); // Skew or rotation (-90 degrees to 0 degrees)

  private double kCameraHeight = LimelightConstants.kCameraHeight;
  private double kTargetHeight = LimelightConstants.kTargetHeight;
  private double n = LimelightConstants.n;
  private double m = LimelightConstants.m;
  private double r1 = LimelightConstants.r1;

  private double theta;
  private double distance;
  private double b;
  private double a;
  private double alpha;
  private double beta;
  
  
  public LimelightSubsystem() {
  
  }


  @Override
  public void periodic() {
    theta = LimelightConstants.mountingAngle + getTy();

    distance = (kTargetHeight-kCameraHeight)/Math.tan(Math.toRadians(theta));

    b = ((kTargetHeight-kCameraHeight)*((-Math.pow(distance,2)*n)-(2*distance*m*r1)+(Math.pow(m,2)*Math.pow(r1,2))))
                          /((distance*m*r1)*((m*r1)-distance));
    a = (((kTargetHeight-kCameraHeight)*(1+n))-(b*(distance-(m*r1))))/(Math.pow((distance-(m*r1)),2));
    alpha = Math.atan((b-Math.tan(Math.toRadians(theta)))/(1+(b*Math.tan(Math.toRadians(theta)))));
    beta = theta + alpha;

    SmartDashboard.putNumber("tv", tv.getDouble(0));
    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.putNumber("ty", ty.getDouble(0));
    SmartDashboard.putNumber("ta", ta.getDouble(0));
    SmartDashboard.putNumber("theta", getTheta());
    SmartDashboard.putNumber("distance", getDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // create accessor methods to reference values outside of this class
  public double getTv() {
    return tv.getDouble(0);
  }
  public double getTx() {
    return tx.getDouble(0);
  }
  public double getTy() {
    return ty.getDouble(0);
  }
  
  public double getTheta() {return theta;}

  public double getDistance() {return distance;}
  public double getA() {return a;}
  public double getAlpha() {return alpha;}
  public double getBeta() {return beta;}

  
  /**
   * set limelight led state
   * @param state Integer with value of 0, 1, 2, or 3
   * <ul>
   * <li>0: use the LED Mode set in the current pipeline</li>
   * <li>1: force off</li>
   * <li>2: force blink</li>
   * <li>3: force on</li>
   * </ul>
   */
  public void setLED(int state) {
    networkTable.getEntry("ledMode").setNumber(state);
  }


}