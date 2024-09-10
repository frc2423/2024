package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make odd LEDs green and even LEDs blue
 * 
 * Things you need to learn:
 * - if/else statements
 * - boolean logic
 */
public class EvenOdd implements Led {
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < length; i++) {
            if(i % 2 == 0){
                buffer.setRGB(i, 00, 255, 00);
            }
            if(i % 2 == 1){
                buffer.setRGB(i, 00, 00, 255);
            }
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
