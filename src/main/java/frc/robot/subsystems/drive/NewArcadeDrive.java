package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NewArcadeDrive extends CommandBase {

  private NewDriveBaseSubsystem driveBaseSubsystem;
  private double kStraight;
  private double kTurn;
  private double kSlowStraight;
  private double kSlowTurn;
  private XboxController joystick;
  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(100);

  public NewArcadeDrive(XboxController joystick, NewDriveBaseSubsystem driveBaseSubsystem, double kStraight, double kTurn) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.kStraight = kStraight;
    this.kTurn = kTurn;
    addRequirements(driveBaseSubsystem);
}

  @Override
  public void initialize() {
    driveBaseSubsystem.factoryResetAll();    
    driveBaseSubsystem.setAllDefaultInversions();
    driveBaseSubsystem.coast(); 
  }

  @Override
  public void execute() {
    boolean squareInputs = true; // square joystick inputs
    double xSpeed = -speedLimiter.calculate(joystick.getRightY() * kStraight);
    double zRotation = rotLimiter.calculate(joystick.getRightX() * kTurn);
    // double xSpeed = -joystick.getLeftY() * kStraight * kSlowStraight;
    // double zRotation = joystick.getRightX() * kTurn * kSlowTurn;
    // double leftPower = kTurn * joystick.getRightX() - kStraight * joystick.getLeftY() + kSlowStraight * joystick.getRightY();
    // double rightPower = -kTurn * joystick.getRightX() - kStraight * joystick.getLeftY() + kSlowStraight * joystick.getRightY();

    driveBaseSubsystem.arcadeDrive(xSpeed, zRotation, squareInputs);
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
