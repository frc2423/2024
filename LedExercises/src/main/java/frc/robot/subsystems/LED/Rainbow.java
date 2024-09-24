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
        for(int i = 0; i<length; i=i+1) {
           buffer.setHSV(i, i*2, 246, 434);    
        }


}

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
