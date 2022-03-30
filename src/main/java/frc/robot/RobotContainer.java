package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.autos.ShootThenMoveAway;
import frc.robot.subsystems.autos.SvrThreeBall;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
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
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();


  // instantiate your auto commands here
  private final ShootThenMoveAway shootThenMoveAway = new ShootThenMoveAway(driveBaseSubsystem, gyroSubsystem, shooterSubsystem, limelightSubsystem, turretSubsystem, feederSubsystem, beamBreakSubsystem, loaderSubsystem);

  // SVR Three Ball Auton
  private final SvrThreeBall svrThreeBall = new SvrThreeBall(turretSubsystem, limelightSubsystem, shooterSubsystem, loaderSubsystem, intakeSubsystem, feederSubsystem, driveBaseSubsystem, gyroSubsystem);
  

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {}

  private void smartDashboardBindings() {}

  public Command getAutonomousCommand() {
    return svrThreeBall;
  }

  // set default commands here
  public void setDefaultCommands() {
  }
}
