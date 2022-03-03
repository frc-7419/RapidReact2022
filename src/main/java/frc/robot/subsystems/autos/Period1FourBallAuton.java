// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.drive.DriveBaseSubsystem;
import frc.robot.subsystems.drive.StraightWithMotionMagic;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.gyro.GyroSubsystem;
import frc.robot.subsystems.gyro.TurnWithGyroClosedLoop;
import frc.robot.subsystems.intake.IntakeSolenoidSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.GetToTargetVelocityWithLimelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.AlignTurret;
import frc.robot.subsystems.turret.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Period1FourBallAuton extends SequentialCommandGroup {
  /** Creates a new Period1ThreeBallForwardBackTurn. */
  public Period1FourBallAuton(DriveBaseSubsystem driveBaseSubsystem, 
                              GyroSubsystem gyroSubsystem, 
                              TurretSubsystem turretSubsystem, 
                              ShooterSubsystem shooterSubsystem, 
                              LimelightSubsystem limelightSubsystem, 
                              IntakeSolenoidSubsystem intakeSolenoidSubsystem, 
                              FeederSubsystem feederSubsystem) {

    //robot drives forward
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 52));

    //robot drives back
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, -80));

    //align turret and shoot
    addCommands(new AlignAndShoot(turretSubsystem, limelightSubsystem, shooterSubsystem, feederSubsystem));
    addCommands(new WaitCommand(0.2));

    //turn 85 degrees clockwise
    addCommands(new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, -85, PIDConstants.GyrokP85, PIDConstants.GyrokI85, PIDConstants.GyrokD85));
    addCommands(new WaitCommand(0.2));

    //drive 240 inches and intake both balls in its path as well as run loader
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, 240));

    //drive back 140 inches to where the ball right outisde of the tarmac was
    addCommands(new StraightWithMotionMagic(driveBaseSubsystem, -140));

    //align turret and shoot
    addCommands(new AlignAndShoot(turretSubsystem, limelightSubsystem, shooterSubsystem, feederSubsystem));

    /*
    Algorithm:
    - Drive dorward and intake first ball
    - Drive back to start and shoot
    - Turn 85 right degrees to the next ball
    - Drive to and intake the next ball and the ball near the hub
    - Drive back to where the previous ball was and shoot
    */
  }
}