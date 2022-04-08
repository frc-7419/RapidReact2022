package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OldArcadeDrive extends CommandBase {

  private OldDriveBaseSubsystem oldDriveBaseSubsystem;
  private double kStraight;
  private double kTurn;
  private double kSlowStraight;
  private double kSlowTurn;
  private XboxController joystick;
  
  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(100);

  public OldArcadeDrive(XboxController joystick, OldDriveBaseSubsystem oldDriveBaseSubsystem, double kStraight, double kTurn) {
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
    if (Math.abs(joystick.getRightX()) > 0 || Math.abs(joystick.getRightY()) > 0) {
      oldDriveBaseSubsystem.coast();
      boolean squareInputs = true; // square joystick inputs
      double xSpeed = -speedLimiter.calculate(joystick.getRightY() * kStraight);
      double zRotation = rotLimiter.calculate(joystick.getRightX() * kTurn);
      oldDriveBaseSubsystem.arcadeDrive(xSpeed, zRotation, squareInputs);
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
