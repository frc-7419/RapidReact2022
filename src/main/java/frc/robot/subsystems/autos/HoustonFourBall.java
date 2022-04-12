// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class HoustonFourBall extends ParallelCommandGroup {

  public HoustonFourBall() {
    sequence(

    );
  }
}

//Face the ball (this ball is our third shot in 3 ball auton) on the field
//Move forward intake ball, turn, shoot 2 balls (preloaded and this one)
//Turn to face the terminal, move forward, intake ball by terminal and human player ball
//Turn back to face hub, move forward, shoot both balls