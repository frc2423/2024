package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make 10 LEDs bounce back and forth from start to end
 */
public class Bouncy implements Led {
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    int min = 0;
    int max = 9;

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        for (var i = min; i < max; i++){
            buffer.setRGB(i, 0, 0, 255);
        }
        max += 1;
        min += 1;
        }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
