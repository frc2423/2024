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

    int t = 0;

    public void run(AddressableLEDBuffer buffer, int length) {
        for (int i = 0; i <= 31; i++){
            buffer.setHSV(i, t, 128, 256);
        }
        t += 1;
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
