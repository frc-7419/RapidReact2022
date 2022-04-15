//period 6 file
package frc.robot.subsystems.autos;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeeder;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.SetLEDColor;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.BrakeTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

public class HoustonTwoBall extends ParallelCommandGroup {

    public HoustonTwoBall(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, LoaderSubsystem loaderSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, LEDSubsystem ledSubsystem, IntakeSolenoidSubsystem intakeSolenoidSubsystem) {
        addCommands(
            sequence(
                // Deploy intake
                new InstantCommand(intakeSolenoidSubsystem::actuateSolenoid, intakeSolenoidSubsystem),
                
                // Drive 46 inches while runing the intake and loader to intake the second cargo
                parallel(
                    new RunIntake(intakeSubsystem, 1),
                    new RunLoader(loaderSubsystem, 0.6)
                ).deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, 46))
                .withTimeout(2.5),

                // Turn 180 degrees while breaking the turret
                new BrakeTurret(turretSubsystem)
                    .deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, 3, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180))
                    .withTimeout(1.5),

                new WaitCommand(0.25),  

                // Allign turret and get to velocity with limelight
                parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem))
                    .withTimeout(0.75), 

                // Allign and shoot with limelight
                new AlignAndShootWithLimelight(turretSubsystem, limelightSubsystem, shooterSubsystem, loaderSubsystem, feederSubsystem),

                new InstantCommand(driveBaseSubsystem::coast, driveBaseSubsystem)
            )
        );
        addCommands(new SetLEDColor(ledSubsystem, limelightSubsystem));
    }
}