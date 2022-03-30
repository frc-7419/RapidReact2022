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
import frc.robot.subsystems.shooter.GetToTargetVelocity;
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

    // start with robot turned towards first ball and turret turned ~80 degrees clockwise
    parallel(
      // run align turret, loader, and intake continuously 
      new AlignTurret(turretSubsystem, limelightSubsystem),
      new RunLoader(loaderSubsystem, 1),
      new RunIntake(intakeSubsystem, 1),

      sequence(
          // Move 80 inches towards first ball
          new StraightWithMotionMagic(driveBaseSubsystem, 80),

          // wait for ball to be intaked
          new WaitCommand(0.3),

          // turn 115 degrees to next ball, while bringing shooter to velocity
          raceWith(
            new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 115, Constants.PIDConstants.GyrokP115, Constants.PIDConstants.GyrokI115, Constants.PIDConstants.GyrokD115),
            new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049) // specific velocity to be tuned
          ),

          // keep shooter at target velocity and run feeder to shoot
          parallel(
            new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049), // specific velocity to be tuned
            new RunFeeder(feederSubsystem, 0.5)
          ).withTimeout(1), // tune the amount of time it takes to shoot both balls

          // short wait between shooting and moving
          new WaitCommand(0.2),

          // move 86 inches while bringing shooter to velocity
          raceWith(
            new StraightWithMotionMagic(driveBaseSubsystem, 86),
            new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049)
          ),

          // keep shooter at target velocity and run feeder to shoot
          parallel(
            new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049), // specific velocity to be tuned
            new RunFeeder(feederSubsystem, 0.5)
          ).withTimeout(1), // tune the amount of time it takes to shoot both balls

          // turn 65 degrees to the second ball, while bringing shooter to velocity
          raceWith(
            new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 65, Constants.PIDConstants.GyrokP63, Constants.PIDConstants.GyrokI63, Constants.PIDConstants.GyrokD63),
            new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049)
          ),

          // keep shooter at target velocity and run feeder to shoot
          parallel(
            new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049), // specific velocity to be tuned
            new RunFeeder(feederSubsystem, 0.5)
          ).withTimeout(1) // tune the amount of time it takes to shoot both balls
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
