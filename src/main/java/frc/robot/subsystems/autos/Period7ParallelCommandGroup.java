package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.autos.Period7Auton;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
public class Period7ParallelCommandGroup extends ParallelCommandGroup {

  public Period7ParallelCommandGroup(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    /* use:
    addCommands(new FooCommand());    addCommands(new BarCommand());
    to add FooCommand and BarCommand to the parallel command group (i.e. run at same time) */
    // addCommands(new Period7Auton(driveBaseSubsystem, gyroSubsystem, turretSubsystem, limelightSubsystem));
    // addCommands(new runIntake());
  }

}
