package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.intake.DeployIntake;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

public class BetterPeriod2Auton extends SequentialCommandGroup {

  public BetterPeriod2Auton(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, IntakeSubsystem intakeSubsystem) {
    //DeployIntake command will be replaced with the RunIntake command once it is fixed
    addCommands(parallel(new Period2Auton(driveBaseSubsystem, gyroSubsystem, turretSubsystem, limelightSubsystem, shooterSubsystem, feederSubsystem), new RunIntake(intakeSubsystem, 0.3)));
  }

}
