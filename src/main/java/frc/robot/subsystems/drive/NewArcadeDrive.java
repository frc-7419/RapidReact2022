// package frc.robot.subsystems.drive;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class NewArcadeDrive extends CommandBase {

//   private BadDriveBaseSubsystem driveBaseSubsystem;
//   private double kStraight;
//   private double kTurn;
//   private double kSlowStraight;
//   private double kSlowTurn;
//   private XboxController joystick;
  
//   // Limits *acceleration* not max speed; basically kD
//   private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);
//   private final SlewRateLimiter rotLimiter = new SlewRateLimiter(100);

//   public NewArcadeDrive(XboxController joystick, BadDriveBaseSubsystem driveBaseSubsystem, double kStraight, double kTurn) {
//     this.joystick = joystick;
//     this.driveBaseSubsystem = driveBaseSubsystem;
//     this.kStraight = kStraight;
//     this.kTurn = kTurn;
//     addRequirements(driveBaseSubsystem);
// }

//   @Override
//   public void initialize() {
//     driveBaseSubsystem.factoryResetAll();    
//     driveBaseSubsystem.setAllDefaultInversions();
//     driveBaseSubsystem.coast(); 
//   }

//   @Override
//   public void execute() {
//     if (Math.abs(joystick.getRightX()) > 0 || Math.abs(joystick.getRightY()) > 0) {
//       driveBaseSubsystem.coast();
//       boolean squareInputs = true; // square joystick inputs
//       double xSpeed = -speedLimiter.calculate(joystick.getRightY() * kStraight);
//       double zRotation = rotLimiter.calculate(joystick.getRightX() * kTurn);
//       driveBaseSubsystem.arcadeDrive(xSpeed, zRotation, squareInputs);
//     }
//     else {
//       driveBaseSubsystem.setAll(0);
//     }
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   @Override
//   public void end(boolean interrupted) {
//     driveBaseSubsystem.setAll(0);
//   }

// }
