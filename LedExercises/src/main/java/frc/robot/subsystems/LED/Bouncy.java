package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make 10 LEDs bounce back and forth from start to end
 */
public class Bouncy implements Led {
    int coloredLedRange = -1;
    boolean reverse = false;
    int currentHue = 0;
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for (var i = 0; i < buffer.getLength(); i++) {
            if (i> coloredLedRange && i<coloredLedRange+9) {
                buffer.setHSV(i, currentHue, 255, 255);
            }
            else{
                buffer.setHSV(i, 0, 0, 0);
            }
        }
        currentHue+=1;
        if (reverse==false && coloredLedRange<length) {
            coloredLedRange+=1;
        }
        if(reverse==true && coloredLedRange>-1) {
            coloredLedRange-=1;
        }
        if(coloredLedRange==-1) {
            reverse=false;
        }
        if(coloredLedRange==60) {
            reverse=true;
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
