package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NewTankDrive extends CommandBase {

  private NewDriveBaseSubsystem newDriveBaseSubsystem;
  private double kStraight;
  private double kSlowStraight;
  private XboxController joystick;
  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter leftSpeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter rightSpeedLimiter = new SlewRateLimiter(2);

  public NewTankDrive(XboxController joystick, NewDriveBaseSubsystem newDriveBaseSubsystem, double kStraight) {
    this.joystick = joystick;
    this.newDriveBaseSubsystem = newDriveBaseSubsystem;
    this.kStraight = kStraight;
    addRequirements(newDriveBaseSubsystem);
}

  @Override
  public void initialize() {
    newDriveBaseSubsystem.factoryResetAll();    
    newDriveBaseSubsystem.setAllDefaultInversions();
    newDriveBaseSubsystem.coast(); 
  }

  @Override
  public void execute() {
    boolean squareInputs = true; // square joystick inputs
    double leftSpeed = leftSpeedLimiter.calculate(joystick.getLeftY() * kStraight);
    double rightSpeed = rightSpeedLimiter.calculate(joystick.getRightY() * kStraight);

    newDriveBaseSubsystem.tankDrive(leftSpeed, rightSpeed, squareInputs);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    newDriveBaseSubsystem.setAll(0);
  }

}
