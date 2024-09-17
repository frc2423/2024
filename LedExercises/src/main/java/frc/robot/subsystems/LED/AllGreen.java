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

    }

    public void run(AddressableLEDBuffer buffer, int length) {

        for (int index = 0; index < length ; index = index + 1) {
            buffer.setRGB(index, 9, 70, 100);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
