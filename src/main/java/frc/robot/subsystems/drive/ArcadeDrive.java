package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
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

  /**
   * reusable arcade command
   * @param joystick
   * @param driveBase
   * @param kStraight
   * @param kTurn
   */
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
    driveBase.coast(); 
    SmartDashboard.putString("command status", "init arcade");
  }

  @Override
  public void execute() {

    SmartDashboard.putString("command status", "exec arcade");
    
    double leftPower = kTurn * joystick.getRightX() - kStraight * joystick.getLeftY() + kSlowStraight * joystick.getRightY();
    double rightPower = -kTurn * joystick.getRightX() - kStraight * joystick.getLeftY() + kSlowStraight * joystick.getRightY();

    // comment out leftX code because it's questionable

    // double leftX = joystick.getLeftX();

    // if(leftX > 0){
    //   rightPower -= kSlowTurn * leftX;
    // }
    // else if(leftX < 0){
    //   leftPower += kSlowTurn * leftX;
    // }
    // else{}

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
