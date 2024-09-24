package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make all LEDs transition through the rainbow
 * 
 * Things you need to learn:
 * - HSV
 */
public class Rainbow implements Led {

    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
    buffer.setHSV(, , 3, 4);
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
