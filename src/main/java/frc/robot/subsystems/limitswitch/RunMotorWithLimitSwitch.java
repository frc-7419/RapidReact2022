package frc.robot.subsystems.limitswitch;
// package frc.robot.subsystems.limitSwitch;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.talon.TalonSubsystem;

// public class RunMotorWithLimitSwitch extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private LimitSwitchSubsystem limitSwitchSubsystem;
//   private TalonSubsystem talonSubsystem;

//   public RunMotorWithLimitSwitch(LimitSwitchSubsystem limitSwitchSubsystem, TalonSubsystem talonSubsystem) {
//     this.limitSwitchSubsystem = limitSwitchSubsystem;
//     this.talonSubsystem= talonSubsystem;
//     // uses addRequirements() instead of requires()
//     addRequirements(limitSwitchSubsystem, talonSubsystem);
//   }

//   @Override
//   public void initialize() {
//   }

//   @Override
//   public void execute() {
//     if (limitSwitchSubsystem.getLimitSwitch().get()){
//       talonSubsystem.setPower(0.2);
//     }
//     // add an else statement that brakes the motor
//     else talonSubsystem.setPower(0);
//   }

//   @Override
//   public void end(boolean interrupted) {
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
