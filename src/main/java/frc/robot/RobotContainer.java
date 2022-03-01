package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.autos.BetterPeriod2Auton;
import frc.robot.subsystems.autos.Period2Auton;
import frc.robot.subsystems.autos.TemplateSequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
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
  public final DriveBaseSubsystem driveBaseSubsystem;
  public final GyroSubsystem gyroSubsystem;
  public final TurretSubsystem turretSubsystem;
  public final LimelightSubsystem limelightSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final FeederSubsystem feederSubsystem;
  public final IntakeSubsystem intakeSubsystem;
  public final BetterPeriod2Auton betterPeriod2Auton;

  //private final Period2Auton period2Auton ;

  // instantiate your auto commands here
  

  public RobotContainer() {
    // Configure the button bindings
    this.driveBaseSubsystem = new DriveBaseSubsystem();
    this.gyroSubsystem = new GyroSubsystem();
    this.turretSubsystem = new TurretSubsystem();
    this.limelightSubsystem = new LimelightSubsystem();
    this.shooterSubsystem = new ShooterSubsystem();
    this.feederSubsystem = new FeederSubsystem();
    this.intakeSubsystem = new IntakeSubsystem();
    this.betterPeriod2Auton = new BetterPeriod2Auton(driveBaseSubsystem, gyroSubsystem, turretSubsystem, limelightSubsystem, shooterSubsystem, feederSubsystem, intakeSubsystem);
    
    //this.period2Auton = new Period2Auton(driveBaseSubsystem, gyroSubsystem);

    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {}

  private void smartDashboardBindings() {}

  public Command getAutonomousCommand() {
    return betterPeriod2Auton;
    //return period2Auton;
  }

  // set default commands here
  public void setDefaultCommands() {
    
  }
}
