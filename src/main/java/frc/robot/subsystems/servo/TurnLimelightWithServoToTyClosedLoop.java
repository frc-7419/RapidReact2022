// package frc.robot.subsystems.servo;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.limelight.LimelightSubsystem;

// public class TurnLimelightWithServoToTyClosedLoop extends CommandBase {
  
//   private ServoSubsystem servoSubsystem;
//   private LimelightSubsystem limelightSubsystem;
//   private PIDController pidController;

//   private double ty;
//   private double tv;
//   private double initAngle = 70;
  
//   private double kP;
//   private double kI;
//   private double kD;

//   private double pidOutput;
  
//   public TurnLimelightWithServoToTyClosedLoop(ServoSubsystem servoSubsystem, LimelightSubsystem limelightSubsystem) {
//     this.servoSubsystem = servoSubsystem;
//     this.limelightSubsystem = limelightSubsystem;
//     addRequirements(servoSubsystem);
//   }

//   @Override
//   public void initialize() {
//     kP = .016;
//     kI = 0;
//     kD = 0; 

//     servoSubsystem.setAngle(initAngle);

//     pidController = new PIDController(kP, kI, kD);
//     pidController.setSetpoint(initAngle + limelightSubsystem.getTy());
//     pidController.setTolerance(0.5);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     ty = limelightSubsystem.getTy();
//     tv = limelightSubsystem.getTv();

//     if (tv == 1.0) {
//       pidOutput = pidController.calculate(ty);
//       servoSubsystem.setAngle(servoSubsystem.getAngle() + pidOutput);
//     }

//     SmartDashboard.putNumber("current angle: ", servoSubsystem.getAngle());
//     SmartDashboard.putNumber("ty +- initial: ", (ty + initAngle));
//     SmartDashboard.putNumber("servo pid output: ", pidOutput);
//     SmartDashboard.putNumber("closed loop error: ", pidController.getPositionError());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // return pidController.atSetpoint();
//     return false;
//   }
// }
