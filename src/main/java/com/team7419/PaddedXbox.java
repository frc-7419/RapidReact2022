package com.team7419;

import edu.wpi.first.wpilibj.XboxController;

public class PaddedXbox extends XboxController{

    public PaddedXbox(){
		super(F310Map.f310Main.value);
    }

    public enum F310Map{
        //Input Map
        f310Main(0),
        f310Secondary(1),
        
        //F310 MAP
        kGamepadAxisLeftStickX(0),
        kGamepadAxisLeftStickY(1),
        kGamepadAxisLeftTrigger(2),
        kGamepadAxisRightTrigger(3),
        kGamepadAxisRightStickX(4),
        kGamepadAxisRightStickY(5),
        kGamepadAxisDpad(6),
        kGamepadButtonA(1),
        kGamepadButtonB(2),
        kGamepadButtonX(3),
        kGamepadButtonY(4),
        kGamepadButtonShoulderL(5),
        kGamepadButtonShoulderR(6),
        kGamepadButtonBack(7),
        kGamepadButtonStart(8),
        kGamepadButtonLeftStickButton(9),
        kGamepadButtonRightStickButton(10),
        kGamepadButtonMode(-1),
        kGamepadButtonLogitech(-1);
        
        public final int value;
        F310Map(int value) {
            this.value = value;
        }	
    }

    public double getRightX() {
		double out = getRawAxis(F310Map.kGamepadAxisRightStickX.value);
		if(Math.abs(out)<.05) {
			out = 0;
		}
		return out;
	}
	public double getLeftX() {
		double out = getRawAxis(F310Map.kGamepadAxisLeftStickX.value);
		if(Math.abs(out)<.05) {
			out = 0;
		}
		return out;
	}
	public double getLeftY() {
		double out = -getRawAxis(F310Map.kGamepadAxisLeftStickY.value);
		if(Math.abs(out)<.05) {
			out = 0;
		}
		return -out;
	}
	public double getRightY() {
		double out = -getRawAxis(F310Map.kGamepadAxisRightStickY.value);
		if(Math.abs(out)<.05) {
			out = 0;
		}
		return out;
	}
	public double getLeftTrig() {
		double out = getRawAxis(F310Map.kGamepadAxisLeftTrigger.value);
		if(Math.abs(out)<.05) {
			out = 0;
		}
		return -out;
	}
	public double getRightTrig() {
		double out = getRawAxis(F310Map.kGamepadAxisRightTrigger.value);
		if(Math.abs(out)<.05) {
			out = 0;
		}
		return out;
	}
	
	public boolean getB() {
		return getRawButton(F310Map.kGamepadButtonB.value);
	}

	
	public boolean getA() {
		return getRawButton(F310Map.kGamepadButtonA.value);
	}
	
	@Override
	public boolean getYButton() {
		return getRawButton(F310Map.kGamepadButtonY.value);
	}
	
	@Override
	public boolean getXButton() {
		return getRawButton(F310Map.kGamepadButtonX.value);
	}

	public boolean getRightShoulder(){
		return getRawButton(F310Map.kGamepadButtonShoulderR.value);
	}

	public boolean getLeftShoulder(){
		return getRawButton(F310Map.kGamepadButtonShoulderL.value);
	}

	public int getDpad(){
		return getPOV();
	}
}