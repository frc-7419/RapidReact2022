//period 6 file
package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShootGetSecondBallShootOneTurn extends ParallelCommandGroup {

    /* comments from Karan:
    you dont need to declare GyroSubsystem since its alr being injected
    we're not using ReadyToShooter, remove it from the constructor, it wont let your code build
    instead of 'null' for the gyro command, substitute it with your instance of GyroSubsystem
    */

    public ShootGetSecondBallShootOneTurn(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, 
    IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, 
    ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, LoaderSubsystem loaderSubsystem, BeamBreakSubsystem beamBreakSubsystem) { 
        addCommands(
            sequence(new StraightWithMotionMagic(driveBaseSubsystem, 67), 
            new WaitCommand(0.25),
            new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180),
            new WaitCommand(0.25)
            )
        );
        addCommands(new RunIntake(intakeSubsystem, 1));
        addCommands(new RunLoader(loaderSubsystem, 1));
        addCommands(new AlignTurretDefault(turretSubsystem, limelightSubsystem));
    }
}
