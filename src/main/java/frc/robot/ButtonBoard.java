package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonBoard extends Joystick {
    
    public ButtonBoard(){
        super(1);
    }

    public double getJoystickX(){
        double out = -getY();
        if(Math.abs(out) < .01){out = 0;}
        return out;
    }
    
    public double getJoystickY(){
        double out = -getX();
        if(Math.abs(out) < .01){out = 0;}
        return out;
    }


}