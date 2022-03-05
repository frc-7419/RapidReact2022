package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.RobotConstants;

public class DriveBaseSubsystem extends SubsystemBase {
  
  private WPI_TalonFX left1;
	private WPI_TalonFX right1;
	private WPI_TalonFX left2;
  private WPI_TalonFX right2;

  private DifferentialDrive drive;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;

  MotorControllerGroup left;
  MotorControllerGroup right;
  
  public DriveBaseSubsystem() {
    left1 = new WPI_TalonFX(CanIds.leftFalcon1.id);
		right1 = new WPI_TalonFX(CanIds.rightFalcon1.id);
		left2 = new WPI_TalonFX(CanIds.leftFalcon2.id);
    right2 = new WPI_TalonFX(CanIds.rightFalcon2.id);

    left = new MotorControllerGroup(left1, left2);
    right = new MotorControllerGroup(right1, right2);

    factoryResetAll();
    right.setInverted(true);

    drive = new DifferentialDrive(left, right);
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(RobotConstants.trackWidth));
    // odometry = new DifferentialDriveOdometry(gyroAngle)
  }

  @Override
  public void periodic() {}

  public enum TurnDirection{
    LEFT,
    RIGHT,
  }

  // accessors
  public TalonFX getLeftMast(){return left1;}
  public TalonFX getRightMast(){return right1;}
  public TalonFX getLeftFollow(){return left2;}
  public TalonFX getRightFollow(){return right2;}
  // motor groups
  public MotorControllerGroup getLeftGroup(){return left;}
  public MotorControllerGroup getRightGroup(){return right;}
  // differential drive
  public DifferentialDrive getDrive(){return drive;}

  public void setLeftPower(double power){
    left.set(power);
  }

  public void setRightPower(double power){
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

  public double getLeftVelocity(){return left1.getSelectedSensorVelocity();}
  public double getRightVelocity(){return right1.getSelectedSensorVelocity();}

  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    drive.arcadeDrive(xSpeed, zRotation, squareInputs);
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
