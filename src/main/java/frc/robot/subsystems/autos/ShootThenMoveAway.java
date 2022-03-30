package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootThenMoveAway extends SequentialCommandGroup {

    public ShootThenMoveAway(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem, FeederSubsystem feederSubsystem, LoaderSubsystem loaderSubsystem) {
        addCommands(
            new GetToTargetVelocity(shooterSubsystem, 9850, 6150, 0.0485, 0.0495).withTimeout(2),
            parallel(new RunLoader(loaderSubsystem, 1.0), new GetToTargetVelocity(shooterSubsystem, 9850, 6150, 0.0485, 0.0495)).withTimeout(2.5),
            new StraightWithMotionMagic(driveBaseSubsystem, -80.88)
        );
    }
    
}
