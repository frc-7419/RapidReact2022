// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public AddressableLED led;
  public AddressableLEDBuffer ledBuffer;
  public int ledLength = 2;

  public LEDSubsystem() {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(ledLength);
    led.setLength(ledBuffer.getLength());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public AddressableLED getLed(){
    return led;
  }

  public AddressableLEDBuffer getLedBuffer(){
    return ledBuffer;
  }

  public void setLEDColor(int red, int green, int blue){
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setRGB(i, red, green, blue);
   }
   led.setData(ledBuffer);
  }

  public void startLed(){
    led.setData(ledBuffer);
    led.start();
  }
  
  public void stopLed(){
    led.setData(ledBuffer);
    led.stop();
  }

  public void rainbowLED(int rainbowFirstPixelHue) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the HSV value to led
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by certain number to make the rainbow "move" (change from 3 to greater number if needed)
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
  }
}
