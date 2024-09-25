package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make all LEDs transition through the rainbow
 * 
 * Things you need to learn:
 * - HSV
 */
public class Rainbow implements Led {
    int currentHue = 0;
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        currentHue+=1;
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, currentHue, 255, 255);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
