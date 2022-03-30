// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import javax.sql.rowset.spi.TransactionalWriter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
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
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SvrThreeBall extends SequentialCommandGroup {
  /** Creates a new SvrThreeBall. */

  public SvrThreeBall(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, 
  ShooterSubsystem shooterSubsystem, LoaderSubsystem loaderSubsystem, IntakeSubsystem intakeSubsystem,
  FeederSubsystem feederSubsystem, DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {

    parallel(
      new AlignTurret(turretSubsystem, limelightSubsystem),
      new RunLoader(loaderSubsystem, 1),
      new RunIntake(intakeSubsystem, 1),

      sequence(
        // Move 80 in
        new StraightWithMotionMagic(driveBaseSubsystem, 80),  
        new RunFeeder(feederSubsystem, 0.5),
        new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 115, Constants.PIDConstants.GyrokP115, Constants.PIDConstants.GyrokI115, Constants.PIDConstants.GyrokD115),
        new GetToTargetVelocityWithLimelight(shooterSubsystem, limelightSubsystem),
        new WaitCommand(0.3),
        new StraightWithMotionMagic(driveBaseSubsystem, 86),
        new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 80, Constants.PIDConstants.GyrokP80, Constants.PIDConstants.GyrokI80, Constants.PIDConstants.GyrokD80),
        new RunFeeder(feederSubsystem, 0.5)
        )
    );
  }
}

// shoot preload
// turn 180
// forward to ball while intake (parallel)
// turn toward third ball
// forward to third ball while intake (parallel)
// turn toward hub
// shoot both
