// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.RunFeeder;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.loader.LoaderSubsystem;
import frc.robot.subsystems.loader.RunLoader;
import frc.robot.subsystems.shooter.GetToTargetVelocity;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.TurnTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

public class SvrThreeBallWithoutIntake extends SequentialCommandGroup {

  public SvrThreeBallWithoutIntake(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, 
  ShooterSubsystem shooterSubsystem, LoaderSubsystem loaderSubsystem,
  FeederSubsystem feederSubsystem, DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    addCommands(
      // start with robot turned towards first ball and turret turned ~80 degrees clockwise
      parallel(
        // run align turret, loader, and intake continuously 
        new AlignTurretDefault(turretSubsystem, limelightSubsystem),
        //new TurnTurret(turretSubsystem, 90),  // can use this in the absense of limelight, otherwise use AlignTurretDefault
        new RunLoader(loaderSubsystem, 0.5),
        //new RunIntake(intakeSubsystem, 1),

        sequence(
            // get to target velocity
            new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049).withInterrupt(() -> shooterSubsystem.bothOnTarget()),
            
            // shoot first ball 
            parallel(
              new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049), // specific velocity to be tuned
              new RunFeeder(feederSubsystem, 0.5)
            ).withTimeout(1), // tune the amount of time it takes to shoot both balls

            // turn 180 degrees towards second ball
            new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, Constants.PIDConstants.GyrokP180, Constants.PIDConstants.GyrokI180, Constants.PIDConstants.GyrokD180),

            // Move 36 inches towards first ball
            new StraightWithMotionMagic(driveBaseSubsystem, 42), // change to specific value

            // wait for ball to be intaked
            new WaitCommand(0.2),

            // turn 115 degrees to next ball
            new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 115, Constants.PIDConstants.GyrokP115, Constants.PIDConstants.GyrokI115, Constants.PIDConstants.GyrokD115),
            
             // move 86 inches while bringing shooter to velocity
            raceWith(
              new StraightWithMotionMagic(driveBaseSubsystem, 117),
              new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049) // specific velocity to be tuned
            ),

            // wait for ball to be intaked
            new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049).withTimeout(0.2),

            new StraightWithMotionMagic(driveBaseSubsystem, -50),

            // keep shooter at target velocity and run feeder to shoot
            parallel(
              new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049), // specific velocity to be tuned
              new RunFeeder(feederSubsystem, 0.5)
            ).withTimeout(1) // tune the amount of time it takes to shoot both balls
        )
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
