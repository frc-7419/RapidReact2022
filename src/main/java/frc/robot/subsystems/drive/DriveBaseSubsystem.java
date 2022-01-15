package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;

public class DriveBaseSubsystem extends SubsystemBase {
  
  public TalonFX left1;
	public TalonFX right1;
	public TalonFX left2;
  public TalonFX right2;
  
  public DriveBaseSubsystem() {
    left1 = new TalonFX(CanIds.leftFalcon1.id);
		right1 = new TalonFX(CanIds.rightFalcon1.id);
		left2 = new TalonFX(CanIds.leftFalcon2.id);
    right2 = new TalonFX(CanIds.rightFalcon2.id);

    factoryResetAll();
    
    right1.setInverted(true);
    right1.setSensorPhase(false);
    right2.setInverted(true);
    right2.setSensorPhase(false);

    left2.follow(left1);
    right2.follow(right1);
    
  }

  @Override
  public void periodic() {
  }

  public enum TurnDirection{
    LEFT,
    RIGHT,
  }

  // accessors
  public TalonFX getLeftMast(){return left1;}
  public TalonFX getRightMast(){return right1;}
  public TalonFX getLeftFollow(){return left2;}
  public TalonFX getRightFollow(){return right2;}

  public void setLeftPower(double power){
    left1.set(ControlMode.PercentOutput, power);
    left2.set(ControlMode.PercentOutput, power);
  }

  public void setRightPower(double power){
    right1.set(ControlMode.PercentOutput, power);
    right2.set(ControlMode.PercentOutput, power);
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

  public void setDefaultInversions() {
    // driveBase.getRightMast().setInverted(true);
    // driveBase.getRightFollow().setInverted(true);
    // driveBase.getLeftMast().setInverted(false);
    // driveBase.getLeftFollow().setInverted(false);
  }

  public void factoryResetAll(){
    right1.configFactoryDefault();
    right2.configFactoryDefault();
    left1.configFactoryDefault();
    left2.configFactoryDefault();
  }
}
