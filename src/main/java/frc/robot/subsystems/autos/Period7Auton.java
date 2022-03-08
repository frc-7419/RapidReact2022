package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class Period7Auton extends SequentialCommandGroup {

  public Period7Auton(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
    /* use:
    addCommands(new FoodCommand());
    addCommands(new BarCommand());
    to add FooCommand and BarCommand to the sequential command group */
    // TODO: change values

    // align and shoot
    addCommands(new AlignAndShoot(turretSubsystem, limelightSubsystem, shooterSubsystem, feederSubsystem, beamBreakSubsystem, 1));

    // Turn to next cargo
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180));

    addCommands(new WaitCommand(0.1));

    // // Drive forward to next cargo
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem,  37.943));

    addCommands(new WaitCommand(0.075));

    // // Turn to the second cargo
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 63.5, PIDConstants.GyrokP63, PIDConstants.GyrokI63, PIDConstants.GyrokD63).withTimeout(5)); //positive equivalent is 147.75 (from -32.25)

    addCommands(new WaitCommand(0.075));

    // // Drive forward to the second cargo
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem,  107.664));

    addCommands(new WaitCommand(0.25));

    // // Turn roughly to the target
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 115, PIDConstants.GyrokP115, PIDConstants.GyrokI115, PIDConstants.GyrokD115).withTimeout(5)); //positive equivalent is 147.75 (from -32.25)

    // Use limight to adjust to the target and shoot
    addCommands(new AlignAndShoot(turretSubsystem, limelightSubsystem, shooterSubsystem, feederSubsystem, beamBreakSubsystem, 2));

  }

}
