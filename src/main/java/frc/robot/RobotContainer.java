package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.autos.Period1AutonPath;
import frc.robot.subsystems.autos.Period1ThreeBallClose;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final XboxController joystick = new XboxController(0);
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  //private final Period1AutonPath period1AutonPath = new Period1AutonPath(driveBaseSubsystem, gyroSubsystem);
  private final Period1ThreeBallClose period1AutonPath = new Period1ThreeBallClose(driveBaseSubsystem, gyroSubsystem);

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {}

  private void smartDashboardBindings() {}

  public Command getAutonomousCommand() {
    return period1AutonPath;
  }

  // set default commands here
  public void setDefaultCommands() {}
}
