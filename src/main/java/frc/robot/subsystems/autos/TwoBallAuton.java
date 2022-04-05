//period 6 file
package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.drive.UnBrake;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeeder;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.BrakeTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TwoBallAuton extends SequentialCommandGroup {

    public TwoBallAuton(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, LoaderSubsystem loaderSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
        addCommands(
            parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocity(shooterSubsystem, 37, 30))
                .withInterrupt(() -> shooterSubsystem.bothOnTarget()).withTimeout(0.5), // gttv while aligning turret

            // shoot preload
            parallel(
                new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                new GetToTargetVelocity(shooterSubsystem, 37, 30),
                new RunFeeder(feederSubsystem, 1),
                new RunLoader(loaderSubsystem, 1)
            ).withTimeout(1.5), // tune time
            
            // new InstantCommand(intakeSolenoidSubsystem::retractSolenoid, intakeSolenoidSubsystem),

            // turn 180 while braking turret
            new BrakeTurret(turretSubsystem)
                .deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 178, 0.5, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180)).withTimeout(2),

            new WaitCommand(0.25),  
            
            // deploy intake
            // new InstantCommand(intakeSolenoidSubsystem::actuateSolenoid, intakeSolenoidSubsystem),

            // move forward and running intake + loader
            parallel(
                new RunIntake(intakeSubsystem, 1),
                new RunLoader(loaderSubsystem, 0.6)
            ).deadlineWith(new StraightWithMotionMagic(driveBaseSubsystem, 50)).withTimeout(2.5),
            
            new WaitCommand(0.25),

            // retract intake
            // new InstantCommand(intakeSolenoidSubsystem::retractSolenoid, intakeSolenoidSubsystem),

            // turn 180 while braking turret
            new BrakeTurret(turretSubsystem)
                .deadlineWith(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, 0.5, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180)).withTimeout(2),

            new WaitCommand(0.25),
            
            parallel(new AlignTurretDefault(turretSubsystem, limelightSubsystem), new GetToTargetVelocity(shooterSubsystem, 45, 43))
                .withInterrupt(() -> shooterSubsystem.bothOnTarget()).withTimeout(0.5), // gttv while aligning turret
            
            // shoot second ball
            parallel(
                new AlignTurretDefault(turretSubsystem, limelightSubsystem),
                new GetToTargetVelocity(shooterSubsystem, 45, 43), // mainting the specific velocity (to be tuned)
                new RunLoader(loaderSubsystem, 1),
                new RunFeeder(feederSubsystem, 0.5)
            ).withTimeout(3),

            new UnBrake(driveBaseSubsystem)
        );
    }
}