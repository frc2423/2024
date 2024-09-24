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
        buffer.setRGB(2, 255, 0, 20);
        buffer.setRGB(0, 255, 0, 20);
        buffer.setRGB(1, 255, 0, 20);
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
