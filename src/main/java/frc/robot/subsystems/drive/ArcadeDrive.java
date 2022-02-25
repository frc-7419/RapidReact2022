package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Reusable arcade command
 */
public class ArcadeDrive extends CommandBase {

  private DriveBaseSubsystem driveBase;
  private double kStraight;
  private double kTurn;
  private double kSlowStraight;
  private double kSlowTurn;
  private XboxController joystick;

  // Limits *acceleration* not max speed; basically kD
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(0.98);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(0.98);

  public ArcadeDrive(XboxController joystick, DriveBaseSubsystem driveBase, double kStraight, double kTurn, double kSlowStraight, double kSlowTurn){
    this.joystick = joystick;
    this.driveBase = driveBase;
    this.kStraight = kStraight;
    this.kTurn = kTurn;
    this.kSlowStraight = kSlowStraight;
    this.kSlowTurn = kSlowTurn;
    addRequirements(driveBase);
}

  @Override
  public void initialize() {
    driveBase.factoryResetAll();    
    driveBase.setAllDefaultInversions();
    driveBase.coast(); 
  }

  @Override
  public void execute() {
    boolean squareInputs = true; // square joystick inputs
    double xSpeed = -speedLimiter.calculate(joystick.getLeftY() * kStraight) * kSlowStraight;
    double zRotation = rotLimiter.calculate(joystick.getRightX() * kTurn) * kSlowTurn;
    // double xSpeed = -joystick.getLeftY() * kStraight * kSlowStraight;
    // double zRotation = joystick.getRightX() * kTurn * kSlowTurn;
    // double leftPower = kTurn * joystick.getRightX() - kStraight * joystick.getLeftY() + kSlowStraight * joystick.getRightY();
    // double rightPower = -kTurn * joystick.getRightX() - kStraight * joystick.getLeftY() + kSlowStraight * joystick.getRightY();

    driveBase.getDrive().arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.setAll(0);
  }

}
