package frc.robot.controllers;


import edu.wpi.first.wpilibj.Joystick;

public class NotVeryOldGuitarHero {
    Joystick guitarHero = new Joystick(2);

    public enum Button {
        kGreen(8),
        kRed(2),
        kYellow(1),
        kBlue(3),
        kOrange(4),
        kSelect(9),
        kStart(10);

        /** Button value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public boolean getGreenButton(){
        return guitarHero.getRawButton(Button.kGreen.value);
    }

    public boolean getRedButton(){
        return guitarHero.getRawButton(Button.kRed.value);
    }

    public boolean getYellowButton(){
        return guitarHero.getRawButton(Button.kYellow.value);
    }

    public boolean getBlueButton(){
        return guitarHero.getRawButton(3);
    }

    public boolean getOrangeButton(){
        return guitarHero.getRawButton(Button.kOrange.value);
    }

    public boolean getSelectButton(){
        return guitarHero.getRawButton(Button.kSelect.value);
    }

    public boolean getStartButton(){
        return guitarHero.getRawButton(Button.kStart.value);
    }

    public boolean getUpStrum(){
        if (guitarHero.getRawAxis(1) == -1){
            return true;
        } else {
            return false;
        }
    }

    public boolean getDownStrum(){
        if (guitarHero.getRawAxis(1) == 1){
            return true;
        } else {
            return false;
        }
    }
}
