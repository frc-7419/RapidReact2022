package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.turret.AlignTurretWithOnboardPIDController;
import frc.robot.subsystems.turret.TurretSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.ArrowRunShooterWithJoystick;
import frc.robot.subsystems.shooter.BasicShooterSubsystem;
import frc.robot.Constants.PowerConstants;
import frc.robot.subsystems.intake.DeployIntake;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.limelight.TurnToTargetClosedLoop;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.spark.JoystickSparkMax;
import frc.robot.subsystems.spark.SparkMaxSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class RobotContainer {
  private final XboxController joystick = new XboxController(0);
  private final SparkMaxSubsystem sparkMaxSubsystem = new SparkMaxSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final JoystickSparkMax joystickSparkMax = new JoystickSparkMax(sparkMaxSubsystem, joystick, 0.1);
  private final TurnToTargetClosedLoop turnToTargetClosedLoop = new TurnToTargetClosedLoop(sparkMaxSubsystem,limelightSubsystem);

  private final BasicShooterSubsystem basicShooterSubsystem = new BasicShooterSubsystem();
  
  private final ArrowRunShooterWithJoystick arrowRunShooterWithJoystick = new ArrowRunShooterWithJoystick(basicShooterSubsystem, joystick);

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeSolenoidSubsystem intakeSolenoidSubsystem = new IntakeSolenoidSubsystem();
  private final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();

  private final DeployIntake deployIntake = new DeployIntake(intakeSolenoidSubsystem, joystick);
  private final RunIntake runIntake = new RunIntake(intakeSubsystem, joystick, PowerConstants.intakeMultiplier);
  private final RunLoader runLoader = new RunLoader(loaderSubsystem, joystick, 1);

  public RobotContainer() {
    configureButtonBindings();

    smartDashboardBindings();
  }

  private void configureButtonBindings() {
    // new JoystickButton(joystick, XboxController.Button.kY.value).toggleWhenPressed(new GetToTargetVelocity(shooterSubsystem, 1000));
   
  }

  private void smartDashboardBindings() {
    // SmartDashboard.putNumber("targetRPM", 1000);
    // SmartDashboard.putNumber("shooterKp", PIDConstants.ShooterkP);
    // SmartDashboard.putNumber("shooterKi", PIDConstants.ShooterkI);
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(0);
  }
    
  public void setDefaultCommands(){
    intakeSubsystem.setDefaultCommand(runIntake);
    intakeSolenoidSubsystem.setDefaultCommand(deployIntake);
    loaderSubsystem.setDefaultCommand(runLoader);
    basicShooterSubsystem.setDefaultCommand(arrowRunShooterWithJoystick);
    sparkMaxSubsystem.setDefaultCommand(joystickSparkMax);
  }
}
