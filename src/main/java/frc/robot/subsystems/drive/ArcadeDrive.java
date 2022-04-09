package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {

  private DriveBaseSubsystem oldDriveBaseSubsystem;
  private double kStraight;
  private double kTurn;
  private XboxController joystick;
  
  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(70);

  public ArcadeDrive(XboxController joystick, DriveBaseSubsystem oldDriveBaseSubsystem, double kStraight, double kTurn) {
    this.joystick = joystick;
    this.oldDriveBaseSubsystem = oldDriveBaseSubsystem;
    this.kStraight = kStraight;
    this.kTurn = kTurn;
    addRequirements(oldDriveBaseSubsystem);
}

  @Override
  public void initialize() {
    oldDriveBaseSubsystem.factoryResetAll();    
    oldDriveBaseSubsystem.setAllDefaultInversions();
    oldDriveBaseSubsystem.coast(); 
  }

  @Override
  public void execute() {
    double xSpeed = -speedLimiter.calculate(joystick.getRightY() * kStraight);
    double zRotation = rotLimiter.calculate(joystick.getRightX() * kTurn);
    
    if (Math.abs(joystick.getRightY()) > 0) {
      oldDriveBaseSubsystem.coast();
      
      // double leftPower = kTurn * joystick.getRightX() + kSlowStraight * joystick.getRightY();
      // double rightPower = -kTurn * joystick.getRightX()+ kSlowStraight * joystick.getRightY();

      double leftPower = xSpeed + zRotation;
      double rightPower = xSpeed - zRotation;
      oldDriveBaseSubsystem.setLeftPower(leftPower);
      oldDriveBaseSubsystem.setRightPower(rightPower);
    }
    else {
      oldDriveBaseSubsystem.setAll(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    oldDriveBaseSubsystem.setAll(0);
  }

}
