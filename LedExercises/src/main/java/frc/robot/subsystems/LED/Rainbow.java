package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make all LEDs transition through the rainbow
 * 
 * Things you need to learn:
 * - HSV
 */
public class Rainbow implements Led {
    public int hue = 0; 
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
          for(int i=0;i<length;i=i+1){
            buffer.setHSV( i,hue, 255, 255);

          }
          hue++;
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
