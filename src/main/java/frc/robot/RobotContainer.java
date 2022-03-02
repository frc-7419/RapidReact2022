package frc.robot;

import com.team7419.joystick.DoubleButton;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.RunShooterWithJoystick;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.RunTurretWithJoystick;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.Constants.PowerConstants;
import frc.robot.commands.RunIntakeAndLoaderWithJoystick;
import frc.robot.subsystems.drive.ArcadeDrive;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.intake.DeployIntakeWithJoystick;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);
  private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final DriveBaseSubsystem driveBaseSubsystem = new DriveBaseSubsystem();
  
  private final DeployIntakeWithJoystick deployIntakeWithJoystick = new DeployIntakeWithJoystick(intakeSolenoidSubsystem, joystick);
  private final RunTurretWithJoystick runTurretWithJoystick = new RunTurretWithJoystick(turretSubsystem, joystick, 0.3);
  private final RunIntakeAndLoaderWithJoystick runIntakeAndLoaderWithJoystick = new RunIntakeAndLoaderWithJoystick(joystick, intakeSubsystem, loaderSubsystem, 1);
  private final AlignTurretDefault alignTurretDefault = new AlignTurretDefault(turretSubsystem, limelightSubsystem);
  private final RunShooterWithJoystick runShooterWithJoystick = new RunShooterWithJoystick(shooterSubsystem, joystick);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(joystick, driveBaseSubsystem, 
  PowerConstants.DriveBaseLeftStraight, PowerConstants.DriveBaseRightTurn, 
  PowerConstants.DriveBaseRightStraight, PowerConstants.DriveBaseLeftTurn);

  public RobotContainer() {
    configureButtonBindings();
    smartDashboardBindings();
  }

  private void configureButtonBindings() {
    // new JoystickButton(joystick, XboxController.Button.kY.value).toggleWhenPressed(new GetToTargetVelocity(shooterSubsystem, 1000, 1000));
    new JoystickButton(joystick, XboxController.Button.kY.value)
      .toggleWhenPressed(new GetToTargetVelocity(shooterSubsystem, 2000, 2000));

  }

  private void smartDashboardBindings() {
    SmartDashboard.putNumber("tTargetRV", 2000);
    SmartDashboard.putNumber("bTargetRV", 2000);
    SmartDashboard.putNumber("ShooterKp", 0);
    SmartDashboard.putNumber("ShooterKi", 0);
    SmartDashboard.putNumber("bKf", 0.05);
    SmartDashboard.putNumber("tKf", 0.05);
  }

  public Command getAutonomousCommand() {
    return alignTurretDefault;
  }
    
  public void setDefaultCommands() {
    driveBaseSubsystem.setDefaultCommand(arcadeDrive);
    intakeSolenoidSubsystem.setDefaultCommand(deployIntakeWithJoystick);
    intakeSubsystem.setDefaultCommand(runIntakeAndLoaderWithJoystick);
    loaderSubsystem.setDefaultCommand(runIntakeAndLoaderWithJoystick); 
    // shooterSubsystem.setDefaultCommand(runShooterWithJoystick);
    turretSubsystem.setDefaultCommand(alignTurretDefault);
  }


}
