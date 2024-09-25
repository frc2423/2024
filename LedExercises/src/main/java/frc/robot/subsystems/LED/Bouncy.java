package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make 10 LEDs bounce back and forth from start to end
 */
public class Bouncy implements Led {
    int coloredLedRange = 0;
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            if (length< coloredLedRange + 9) {
                if (i> coloredLedRange) {
                    buffer.setRGB(i, 0, 0,255);
                }
            }
            else {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
        coloredLedRange+=1;
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
