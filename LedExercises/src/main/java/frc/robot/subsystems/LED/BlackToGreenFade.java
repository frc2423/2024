package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Make the first LED fade from black to green over time.
 * 
 * Things you need to learn:
 * - variables (you'll probably want to use a class member variable to complete this challenge)
 */
public class BlackToGreenFade implements Led {
   public int counter = 0; 
    public void start(AddressableLEDBuffer buffer, int length) {
    }

    public void run(AddressableLEDBuffer buffer, int length) {
    counter = counter +1; 
    for (var i = 0; i < buffer.getLength(); i++){
        System.out.println(counter);
        buffer.setRGB(i,0,counter,0);
    }}

    public void end(AddressableLEDBuffer buffer, int length) {

    }
}

