//period 6 file
package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.PIDConstants;
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
import frc.robot.subsystems.turret.AlignTurret;
import frc.robot.subsystems.turret.AlignTurretDefault;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TwoBallAuton extends ParallelCommandGroup {

    /* comments from Karan:
    you dont need to declare GyroSubsystem since its alr being injected
    we're not using ReadyToShooter, remove it from the constructor, it wont let your code build
    instead of 'null' for the gyro command, substitute it with your instance of GyroSubsystem
    */

    public TwoBallAuton(DriveBaseSubsystem driveBaseSubsystem, GyroSubsystem gyroSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, LoaderSubsystem loaderSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) { //add parameters
        //Robot is initially facing the hub. We then shoot the ball. Next we will turn the robot so that it can go back
        //and collect the second ball and then shoot it
        //addCommands(new AlignTurret(turretSubsystem, limelightSubsystem));
        addCommands(sequence(
            new GetToTargetVelocity(shooterSubsystem, 7900*1, 9900*1, 0.04874, 0.049).withTimeout(2.1), // mainting the specific velocity (to be tuned));
            parallel(
                new GetToTargetVelocity(shooterSubsystem, 7900*1, 9900*1, 0.04874, 0.049), // mainting the specific velocity (to be tuned)
                new RunFeeder(feederSubsystem, 1),
                new RunLoader(loaderSubsystem, 1)
            ).withTimeout(1.5),
            new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180), //180 degree turn. 
            //Decorator where if the command doesn't finish in that time interval it will move on
            new WaitCommand(0.1),
            race(
                new StraightWithMotionMagic(driveBaseSubsystem, 50),
                new RunLoader(loaderSubsystem, 0.6)
            ).withTimeout(3), //The robot will ideally be positioned toward
            //the middle of the tarmac so it will have to move straight about half of the distance between the hub and the ball
            //to reach the ball
            new WaitCommand(0.2),

            //Here, we will collect the ball and then turn around and then shoot it 
            new TurnWithGyroClosedLoop(driveBaseSubsystem, gyroSubsystem, 180, PIDConstants.GyrokP180, PIDConstants.GyrokI180, PIDConstants.GyrokD180), //180 degree turn
            new WaitCommand(0.25),
            //new StraightWithMotionMagic(driveBaseSubsystem, 50)); //The robot will ideally be positioned toward
            //here after moving 58.08 inches, it will return back to its orignal position and then shoot
            new GetToTargetVelocity(shooterSubsystem, 7900*1.25, 9900*1.25, 0.04874, 0.049).withInterrupt(() -> shooterSubsystem.bothOnTarget()), // mainting the specific velocity (to be tuned)
            
            parallel(
                new RunLoader(loaderSubsystem, 1),
                new GetToTargetVelocity(shooterSubsystem, 7900*1.4, 9900*1.4, 0.04874, 0.049), // mainting the specific velocity (to be tuned)
                new RunFeeder(feederSubsystem, 0.5)
            ).withTimeout(3),
            //changed the kD to 0.0001685
            new WaitCommand(5)
        ));
        addCommands(new RunIntake(intakeSubsystem, 1));
        addCommands(new AlignTurretDefault(turretSubsystem, limelightSubsystem));
    }
}
