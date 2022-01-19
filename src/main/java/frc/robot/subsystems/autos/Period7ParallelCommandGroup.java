package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.autos.Period7Auton;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
public class Period7ParallelCommandGroup extends ParallelCommandGroup {

  public Period7ParallelCommandGroup(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    /* use:
    addCommands(new FooCommand());    addCommands(new BarCommand());
    to add FooCommand and BarCommand to the parallel command group (i.e. run at same time) */
    addCommands(new Period7Auton(driveBaseSubsystem, gyroSubsystem));
    // addCommands(new runIntake());
  }

}
