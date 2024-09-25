package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make 10 LEDs bounce back and forth from start to end
 */
public class Bouncy implements Led {
    int first = 0;
    int length = 10;
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
        for(int i=0;i<10;i=i+1){
            buffer.setRGB(i, 255, 0, 20);
        }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
