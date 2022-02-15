package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;

public class Period7Auton extends SequentialCommandGroup {

  public Period7Auton(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    /* use:
    addCommands(new FoodCommand());
    addCommands(new BarCommand());
    to add FooCommand and BarCommand to the sequential command group */
    // TODO: change values

    // Adjust the robot to turn to hub
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 10));

    // Shoot preloaded balls
    // addCommands(new ShootAtAngle());

    // Turn to next cargo
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 63.5));

    // Drive forward to next cargo
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem,  37.943));

    // Turn to the second cargo
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -32.25)); //positive equivalent is 147.75 (from -32.25)

    // Drive forward to the second cargo
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem,  107.664));

    // Turn roughly to the target
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 80.25));

    // Use limight to adjust to the target
    // addCommands(new TurnToTx(driveBaseSubsystem, limelightSubsystem));

    // Shoot two cargos
    // addCommands(new ShootAtAngle());

  }

}
