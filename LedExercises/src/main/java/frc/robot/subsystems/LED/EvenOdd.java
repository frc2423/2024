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
        for (var i = 0; i < buffer.getLength(); i++){
        if(i%2==0){
            buffer.setRGB(i, 0, 255, 0);
        }else{
            buffer.setRGB(i,0,0,255);
        }
    }
    }

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}
