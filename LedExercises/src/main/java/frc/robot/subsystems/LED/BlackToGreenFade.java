package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make the first LED fade from black to green over time.
 * 
 * Things you need to learn:
 * - variables (you'll probably want to use a class member variable to complete this challenge)
 */
public class BlackToGreenFade implements Led {
        int greenID = 0;

    public void start(AddressableLEDBuffer buffer, int length) {
    }


    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < 31; i++){
            buffer.setRGB(i, 0, greenID, 0);
        }
                    greenID +=1;


    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
