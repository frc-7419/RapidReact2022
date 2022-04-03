package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDrive extends CommandBase {

  private DriveBaseSubsystem driveBaseSubsystem;
  private double kStraight;
  private double kTurn;
  private XboxController joystick;

  
  public ArcadeDrive(XboxController joystick, DriveBaseSubsystem driveBaseSubsystem, double kStraight, double kTurn) {
    this.joystick = joystick;
    this.driveBaseSubsystem = driveBaseSubsystem;
    this.kStraight = kStraight;
    this.kTurn = kTurn;
    addRequirements(driveBaseSubsystem);
}

  @Override
  public void initialize() {
    // driveBaseSubsystem.factoryResetAll();    
    driveBaseSubsystem.coast(); 
  }

  @Override
  public void execute() {
    double leftPower = kTurn * joystick.getRightX() - kStraight * joystick.getRightY();
    double rightPower = -kTurn * joystick.getRightX() - kStraight * joystick.getRightY();

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
