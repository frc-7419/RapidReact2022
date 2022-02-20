// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Period1ThreeBallForwardBackTurn extends SequentialCommandGroup {
  /** Creates a new Period1ThreeBallForwardBackTurn. */
  public Period1ThreeBallForwardBackTurn() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /*
    Algorithm:
    - Drive dorward and intake first ball
    - Drive back to start and shoot
    - Turn 90 degrees to the next ball
    - Drive to the next ball
    - Intake and shoot
    */

  }
}
