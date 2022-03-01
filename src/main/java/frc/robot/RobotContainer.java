package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.autos.ShootThenMoveAway;
import frc.robot.subsystems.autos.TemplateSequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transferWheel.TransferWheelSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController joystick = new XboxController(0);
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  private final GyroSubsystem gyroSubsystem = new GyroSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final TransferWheelSubsystem transferWheelSubsystem = new TransferWheelSubsystem();

  // instantiate your auto commands here
  private final TemplateSequentialCommandGroup templateSequentialCommandGroup = new TemplateSequentialCommandGroup();
  private final ShootThenMoveAway shootThenMoveAway = new ShootThenMoveAway(driveBaseSubsystem, gyroSubsystem, shooterSubsystem, limelightSubsystem, turretSubsystem, transferWheelSubsystem);
  

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {}

  private void smartDashboardBindings() {}

  public Command getAutonomousCommand() {
    return shootThenMoveAway;
  }

  // set default commands here
  public void setDefaultCommands() {
    
  }
}
