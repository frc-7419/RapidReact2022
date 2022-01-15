// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // enums are a data structure primarilly used for constants
    public static enum CanIds {
        
        //to add a motor's can id: motorName(canId),
        // motor1(5),
        // motor2(3),
        // motorX(x)...

        cameraServo(1), // fix this

        // 2020 robot constants
        leftFalcon1(5),
        rightFalcon1(2),
        leftFalcon2(4),
        rightFalcon2(3),
        loaderFalcon(10),
        intakeVictor(11),
        revolverVictor(12),
        climberFalcon(13),
        shooterFalcon(14), 
        hoodVictor(40), 
        ;

        public final int id;

        private CanIds(int id) {
            this.id = id;
        }

    }

    public static double kTargetHeight = 80; // 98 ish in real game
    
    public static class RobotConstants{
        public static double kCameraHeight = 10;
    }

}
