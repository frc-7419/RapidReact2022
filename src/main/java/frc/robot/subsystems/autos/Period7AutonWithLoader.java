// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.intake.RunIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Period7AutonWithLoader extends SequentialCommandGroup {
  /** Creates a new Period7AutonWithLoader. 
 * @param driveBaseSubsystem 
 * @param gyroSubsystem 
 * @param turretSubsystem 
 * @param limelightSubsystem 
 * @param shooterSubsystem 
 * @param feederSubsystem 
 * @param intakeSubsystem 
 * @param loaderSubsystem */
  public Period7AutonWithLoader(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, IntakeSubsystem intakeSubsystem, LoaderSubsystem loaderSubsystem, BeamBreakSubsystem beamBreakSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(parallel(new Period7Auton(driveBaseSubsystem, gyroSubsystem, turretSubsystem, limelightSubsystem, shooterSubsystem, feederSubsystem, beamBreakSubsystem), new RunIntake(intakeSubsystem, 1), new RunLoader(loaderSubsystem, 1)));
  }
}
