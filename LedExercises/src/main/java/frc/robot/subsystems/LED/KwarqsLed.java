package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KwarqsLed extends SubsystemBase {
    private final LedController ledController = new LedController(60); // 36 on each side

    public KwarqsLed() {
        ledController.add("orange", new Orange());
        ledController.add("purple", new Purple());
        ledController.add("rainbow", new Rainbow());
        ledController.add("dark", new Dark());

        setDefaultCommand(disable());
    }

    public Command disable() {
        return run(() -> ledController.set(("dark"))).withName("dark");
    }

    public Command setOrange() {
        return run(() -> ledController.set(("orange"))).withName("orange");
    }

    public Command setPurple() {
        return run(() -> ledController.set(("purple"))).withName("purple");
    }

    public Command setGreen() {
        return run(() -> ledController.set(("green"))).withName("green");
    }

    @Override
    public void periodic() {
        ledController.run();
    }

}