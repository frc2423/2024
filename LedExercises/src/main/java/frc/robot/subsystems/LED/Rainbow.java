package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make all LEDs transition through the rainbow
 * 
 * Things you need to learn:
 * - HSV
 */
public class Rainbow implements Led {
    int r = 255;
    int g = 0;
    int b = 0;
    boolean goingDownR = true;

    public void start(AddressableLEDBuffer buffer, int length) {
    }
    
    public void run(AddressableLEDBuffer buffer, int length) {
        if (r==0){
            goingDownR=false;
        }
        
        if (r==255){
            goingDownR=true;
        }

        if (goingDownR == false) {
            b-=5;
            r+=5;
        } else {
            r-=5;
            g+=5;
        }
        
        for (var i = 0; i<length; i++) {
            buffer.setRGB(i,r,g,b);
        }

    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
