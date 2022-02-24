package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.PowerConstants;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);

  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick, driveBaseSubsystem, 
  PowerConstants.DriveBaseLeftStraight, PowerConstants.DriveBaseRightTurn, 
  PowerConstants.DriveBaseRightStraight, PowerConstants.DriveBaseLeftTurn);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }

  public void setDefaultCommands(){
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
  }

  
}
