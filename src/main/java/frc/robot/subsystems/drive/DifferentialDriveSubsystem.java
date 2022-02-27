package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;
import frc.robot.Constants.CanIds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.team7419.math.UnitConversions;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class DifferentialDriveSubsystem extends SubsystemBase {
  
  // Talons
  public WPI_TalonFX left1;
	public WPI_TalonFX right1;
	public WPI_TalonFX left2;
  public WPI_TalonFX right2;

  // Gyro
  public AHRS ahrs; 

  // Motor Controller Group 
  private final MotorControllerGroup m_leftMotors; 
  private final MotorControllerGroup m_rightMotors;

  private final DifferentialDrive m_drive;

  // // The left-side drive encoder
  // private final Encoder m_leftEncoder = 
  //     new Encoder(
  //         DriveConstants.kLeftEncoderPorts[0],
  //         DriveConstants.kLeftEncoderPorts[1],
  //         DriveConstants.kLeftEncoderReversed);

  // // The right-side drive encoder
  // private final Encoder m_rightEncoder =
  //     new Encoder(
  //         DriveConstants.kRightEncoderPorts[0],
  //         DriveConstants.kRightEncoderPorts[1],
  //         DriveConstants.kRightEncoderReversed);  

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DifferentialDriveSubsystem() {
    this.right1 = new WPI_TalonFX(CanIds.rightFalcon1.id);
    this.right2 = new WPI_TalonFX(CanIds.rightFalcon2.id);
    this.left1 = new WPI_TalonFX(CanIds.leftFalcon1.id);
    this.left2 = new WPI_TalonFX(CanIds.leftFalcon1.id);

    this.m_leftMotors = new MotorControllerGroup(left1, left2);
    this.m_rightMotors = new MotorControllerGroup(right1, right2);
    this.m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    ahrs = new AHRS(SerialPort.Port.kMXP);

    m_rightMotors.setInverted(true);
    
    //Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        ahrs.getRotation2d(), getAverageLeftDistance(), getAverageRightDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getAverageLeftVelocity() {
    return UnitConversions.rawSensorVelocityToMPS((this.left1.getSelectedSensorVelocity() + this.left2.getSelectedSensorVelocity())/2.0, 3, 10.71, 2048);
  }

  public double getAverageRightVelocity() {
    return UnitConversions.rawSensorVelocityToMPS((this.right1.getSelectedSensorVelocity() + this.right2.getSelectedSensorVelocity())/2.0, 3, 10.71, 2048);
  }
  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getAverageLeftVelocity(), getAverageRightVelocity());
  }


  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, ahrs.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    this.right1.setSelectedSensorPosition(0);
    this.right2.setSelectedSensorPosition(0);
    this.left1.setSelectedSensorPosition(0);
    this.left2.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    return UnitConversions.ticksToMeters((this.right1.getSelectedSensorPosition() + this.right2.getSelectedSensorPosition() + this.right1.getSelectedSensorPosition() + this.left2.getSelectedSensorPosition())/4.0, 3, 10.71, 2048);
  }

  public double getAverageRightDistance() {
    return UnitConversions.ticksToMeters((this.right1.getSelectedSensorPosition() + this.right2.getSelectedSensorPosition())/2.0, 3, 10.71, 2048);
  }

  public double getAverageLeftDistance() {
    return UnitConversions.ticksToMeters((this.left1.getSelectedSensorPosition() + this.left2.getSelectedSensorPosition())/2.0, 3, 10.71, 2048);
  }
  // /**
  //  * Gets the left drive encoder.
  //  *
  //  * @return the left drive encoder
  //  */
  // public Encoder getLeftEncoder() {
  //   return m_leftEncoder;
  // }

  // /**
  //  * Gets the right drive encoder.
  //  *
  //  * @return the right drive encoder
  //  */
  // public Encoder getRightEncoder() {
  //   return m_rightEncoder;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return ahrs.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -ahrs.getRate();
  }
}