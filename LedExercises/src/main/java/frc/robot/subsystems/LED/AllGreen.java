package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make all LEDs green
 * 
 * Things you need to learn:
 * - for loops
 */
public class AllGreen implements Led {
    public void start(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 00, 255, 00);
        }
    }

    public void run(AddressableLEDBuffer buffer, int length) {

    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
