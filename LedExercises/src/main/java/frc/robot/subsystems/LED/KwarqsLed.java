package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KwarqsLed extends SubsystemBase {
    private final LedController ledController = new LedController(60); // 36 on each side

    public KwarqsLed() {
        ledController.add("Dark", new Dark());
        ledController.add("Orange", new Orange());
        ledController.add("First3Green", new First3Green());
        ledController.add("AllGreen", new AllGreen());
        ledController.add("EvenOdd", new EvenOdd());
        ledController.add("BlackToGreenFade", new BlackToGreenFade());
        ledController.add("Rainbow", new Rainbow());
        ledController.add("Bouncy", new Bouncy());

        setDefaultCommand(disable());
    }

    public Command disable() {
        return run(() -> ledController.set(("Dark"))).withName("Disable");
    }

    public Command setOrange() {
        return run(() -> ledController.set(("Orange"))).withName("Orange");
    }

    public Command setFirst3Green() {
        return run(() -> ledController.set(("First3Green"))).withName("First 3 Green");
    }

    public Command setAllGreen() {
        return run(() -> ledController.set(("AllGreen"))).withName("All Green");
    }

    public Command setEvenOdd() {
        return run(() -> ledController.set(("EvenOdd"))).withName("Even Odd");
    }

    public Command setBlackToGreenFade() {
        return run(() -> ledController.set(("BlackToGreenFade"))).withName("Black to Green Fade");
    }

    public Command setRainbow() {
        return run(() -> ledController.set(("Rainbow"))).withName("Rainbow");
    }

    public Command setBouncy() {
        return run(() -> ledController.set(("Bouncy"))).withName("Bouncy");
    }

    @Override
    public void periodic() {
        ledController.run();
    }

}