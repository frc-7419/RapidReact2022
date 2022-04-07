package frc.robot.subsystems.autos;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.FollowTrajectory;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.loader.LoaderSubsystem;

public class PathTestAuton extends SequentialCommandGroup {
    public PathTestAuton(DriveBaseSubsystem driveBaseSubsystem, IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem) {
        addCommands(
            parallel(
                new FollowTrajectory(driveBaseSubsystem, PathPlanner.loadPath(String.format(
                        "autontest"), DriveConstants.maxVelocity, DriveConstants.maxAcceleration)),
                new RunIntake(intakeSubsystem, 1),
                new RunLoader(loaderSubsystem, 0.6)
            )
        );
        
    }
}
