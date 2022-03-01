package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private double kStraight;
  private double kTurn;
  private double kSlowStraight;
  private double kSlowTurn;
  private XboxController joystick;

  
  public ArcadeDrive(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem, double kStraight, double kTurn, double kSlowStraight, double kSlowTurn){
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.kStraight = kStraight;
    this.kTurn = kTurn;
    this.kSlowStraight = kSlowStraight;
    this.kSlowTurn = kSlowTurn;
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
    double leftPower = kTurn * joystick.getRightX() - kStraight * joystick.getLeftY() + kSlowStraight * joystick.getRightY();
    double rightPower = -kTurn * joystick.getRightX() - kStraight * joystick.getLeftY() + kSlowStraight * joystick.getRightY();

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
