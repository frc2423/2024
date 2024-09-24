package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make the first 3 LEDs green
 * 
 * Things you need to learn:
 * - functions
 * - RGB (red, green, blue)
 */
public class First3Green implements Led {
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        // buffer.setRGB(0,25,0,245[188,15,0,0]);

        buffer.setRGB(0,214,255,110);
        buffer.setRGB(1 ,10,226,100);
    
    buffer.setRGB(2 ,2,255,1);
    }
    
    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
