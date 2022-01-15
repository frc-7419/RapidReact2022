package frc.robot.subsystems.drive;

import frc.robot.PaddedXbox;
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
  private PaddedXbox joystick;

  
  public ArcadeDrive(PaddedXbox joystick, DriveBaseSubsystem driveBase, double kStraight, double kTurn, double kSlowStraight, double kSlowTurn){
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
    double leftPower = kTurn * joystick.getRightX() - kStraight * joystick.getLeftY() + kSlowStraight * joystick.getRightY();
    double rightPower = -kTurn * joystick.getRightX() - kStraight * joystick.getLeftY() + kSlowStraight * joystick.getRightY();

    driveBase.setLeftPower(leftPower);
    driveBase.setRightPower(rightPower);
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
