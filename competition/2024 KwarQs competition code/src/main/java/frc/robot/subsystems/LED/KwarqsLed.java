package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionSubsystem;

public class KwarqsLed extends SubsystemBase {
    private final LedController ledController = new LedController(60); // 36 on each side
    private final VisionSubsystem visionSubsystem;

    public KwarqsLed(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        ledController.add("yellow", new Yellow());
        ledController.add("purple", new Purple());
        ledController.add("green", new Green());
        ledController.add("dark", new Dark());

        // setDefaultCommand(disable());
    }

    public Command disable() {
        var command = Commands.run(() -> {
                        // System.out.println("SEEES DARK!!!");

            // ledController.set("dark");
        });
        // command.ignoringDisable(true);

        command.addRequirements(this);
        return command;
    }

    public Command setYellow() {
        var command = Commands.run(() -> {
            // System.out.println("SEEES YELLOW!!!");
            // ledController.set("yellow");
        });
        // command.ignoringDisable(?true);
        command.addRequirements(this);
        return command;
    }

    public Command setPurple() {
        var command = Commands.run(() -> {
            // ledController.set("purple");
        });
        command.addRequirements(this);
        return command;
    }

    public Command setGreen() {
        var command = Commands.run(() -> {
            // System.out.println("!!!!!!!!");
            // ledController.set("green");
        });
        command.addRequirements(this);
        return command;
    }

    @Override
    public void periodic() {

        if (RobotState.isDisabled() && visionSubsystem.seesAprilTag()) {
            ledController.set("yellow");
        } else {
            ledController.set("dark");
        }
        ledController.run();
    }
}