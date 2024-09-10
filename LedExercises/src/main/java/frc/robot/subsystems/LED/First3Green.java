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
        for (var i = 0; i < 3; i++) {
            buffer.setRGB(i, 00, 255, 00);
        }

    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}