package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDrive extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private double kStraight;
  private XboxController joystick;

  
  public TankDrive(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem, double kStraight) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.kStraight = kStraight;
    addRequirements(driveBaseSubsystem);
}

  @Override
  public void initialize() {
    // driveBaseSubsystem.factoryResetAll();    
    driveBaseSubsystem.coast(); 
  }

  @Override
  public void execute() {
    double leftPower = kStraight * joystick.getLeftY();
    double rightPower = kStraight * joystick.getRightY();

    driveBaseSubsystem.setLeftPower(leftPower);
    driveBaseSubsystem.setRightPower(rightPower);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveBaseSubsystem.setAll(0);
  }

}
