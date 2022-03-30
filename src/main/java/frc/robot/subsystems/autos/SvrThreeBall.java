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
import frc.robot.subsystems.turret.TurretSubsystem;

public class SvrThreeBall extends SequentialCommandGroup {

  public SvrThreeBall(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem, 
  ShooterSubsystem shooterSubsystem, LoaderSubsystem loaderSubsystem, IntakeSubsystem intakeSubsystem,
  FeederSubsystem feederSubsystem, DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem) {
    addCommands(
      // start with robot turned towards first ball and turret turned ~80 degrees clockwise
      parallel(
        // run align turret, loader, and intake continuously 
        new AlignTurretDefault(turretSubsystem, limelightSubsystem),
        new RunLoader(loaderSubsystem, 1),
        new RunIntake(intakeSubsystem, 1),

        sequence(
            // robot initially faces the scoring hub

            // wait for shooter to reach target velocity
            new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049),

            // shoot preload
            new RunFeeder(feederSubsystem, 0.5),

            // turn 180 degrees to face the second ball
            new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, Constants.PIDConstants.GyrokP180, Constants.PIDConstants.GyrokI180, Constants.PIDConstants.GyrokD180),
            
            // Move __ inches towards second ball --> NOTE: STILL NEED TO CALCULATE PRELOAD TO SECOND BALL DISTANCE, not 80 in
            new StraightWithMotionMagic(driveBaseSubsystem, 80),

            // wait for ball to be intaked
            new WaitCommand(0.2),

            new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 115, Constants.PIDConstants.GyrokP115, Constants.PIDConstants.GyrokI115, Constants.PIDConstants.GyrokD115),

            // keep shooter at target velocity and run feeder to shoot
            new StraightWithMotionMagic(driveBaseSubsystem, 114),

            // wait for ball to be intaked
            new WaitCommand(0.2),

            // get to target velocity as the robot turns 65 degrees
            raceWith(
              new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 63, Constants.PIDConstants.GyrokP63, Constants.PIDConstants.GyrokI63, Constants.PIDConstants.GyrokD63),
              new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049) // specific velocity to be tuned
            ),

            // easier to get to target velocity from the wheel's angular momentum from the previous run and run feeder to shoot
            raceWith(
              new GetToTargetVelocity(shooterSubsystem, 7900, 9900, 0.04874, 0.049),
              new RunFeeder(feederSubsystem, 0.5)
            )
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
