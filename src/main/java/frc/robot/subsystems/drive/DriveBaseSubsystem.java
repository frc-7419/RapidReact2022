package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.CanIds;
import frc.robot.Constants.RobotConstants;

public class DriveBaseSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX left1, right1, left2, right2;
  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  private final DifferentialDrive drive;
  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

  private final MotorControllerGroup left;
  private final MotorControllerGroup right;
  
  public DriveBaseSubsystem() {
    left1 = new WPI_TalonFX(CanIds.leftFalcon1.id);
		right1 = new WPI_TalonFX(CanIds.rightFalcon1.id);
		left2 = new WPI_TalonFX(CanIds.leftFalcon2.id);
    right2 = new WPI_TalonFX(CanIds.rightFalcon2.id);

    left = new MotorControllerGroup(left1, left2);
    right = new MotorControllerGroup(right1, right2);

    factoryResetAll();
    setAllDefaultInversions();

    drive = new DifferentialDrive(left, right);
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(RobotConstants.trackWidth));
    resetEncoders();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());
  }
  public enum TurnDirection {
    LEFT,
    RIGHT,
  }

  public TalonFX getLeftMast(){return left1;}
  public TalonFX getRightMast(){return right1;}
  public TalonFX getLeftFollow(){return left2;}
  public TalonFX getRightFollow(){return right2;}


  public MotorControllerGroup getLeftGroup(){return left;}
  public MotorControllerGroup getRightGroup(){return right;}
  public DifferentialDrive getDrive(){return drive;}
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }
  public double getLeftDistance() {
    return left1.getSelectedSensorPosition()/2048;
  }
  public double getRightDistance() {
    return right1.getSelectedSensorPosition()/2048;
  }

  public double getLeftVelocity() {
    return left1.getSelectedSensorVelocity();
  }
  public double getRightVelocity() {
    return right1.getSelectedSensorVelocity();
  }
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity()/2048, getRightVelocity()/2048);
  }

  public void setLeftPower(double power) {
    left.set(power);
  }

  public void setRightPower(double power) {
    right.set(power);
  }

  public void setAll(double power){
    setLeftPower(power);
    setRightPower(power);
  }

  public void stop(){setAll(0);}

  public void setAllMode(NeutralMode mode){
    right1.setNeutralMode(mode);
    right2.setNeutralMode(mode);
    left1.setNeutralMode(mode);
    left2.setNeutralMode(mode);
  }

  public void brake(){setAllMode(NeutralMode.Brake);}

  public void coast(){setAllMode(NeutralMode.Coast);}

  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    drive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetEncoders() {
    left1.setSelectedSensorPosition(0);
    left2.setSelectedSensorPosition(0);
    right1.setSelectedSensorPosition(0);
    right2.setSelectedSensorPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void setAllDefaultInversions() {
    right.setInverted(true);
    left.setInverted(false);
  }

  public void factoryResetAll(){
    right1.configFactoryDefault();
    right2.configFactoryDefault();
    left1.configFactoryDefault();
    left2.configFactoryDefault();
  }
}
