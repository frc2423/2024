package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make the first LED fade from black to green over time.
 * 
 * Things you need to learn:
 * - variables (you'll probably want to use a class member variable to complete this challenge)
 */
public class BlackToGreenFade implements Led {
    int green = 0;
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        if (green < 255) {
            green = green + 1;
        }
        buffer.setRGB(0 ,0,green,0);

        /*
        for (int green = 0; green <256; green=green+1) {
            buffer.setRGB(0 ,0,green,0);
        }
        */
    }    

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
