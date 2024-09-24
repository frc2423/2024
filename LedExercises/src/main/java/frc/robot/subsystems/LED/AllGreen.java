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
        for(int i=0;i<length;i=i+1){
            buffer.setRGB(i, 0, 255, 0);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
