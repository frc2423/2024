package frc.robot.subsystems.LED;

import javax.management.ServiceNotFoundException;

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
        for (int index = 0; index < length ; index = index + 8) {
            buffer.setRGB(index, 11, 28, 212);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
