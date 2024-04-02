package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NTHelper;
import frc.robot.subsystems.vision.VisionSubsystem;

public class KwarqsLed extends SubsystemBase {
    private final LedController ledController = new LedController(60); // 36 on each side
    private final VisionSubsystem visionSubsystem;

    public KwarqsLed(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;
        ledController.add("yellow", new Yellow());
        ledController.add("orange", new Orange());
        ledController.add("purple", new Purple());
        ledController.add("green", new Green());
        ledController.add("rainbow", new Rainbow());
        ledController.add("dark", new Dark());

        NTHelper.setString("/sourcePickUp", "none");

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

    public Command setOrange() {
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

    public boolean isGroundPickUp() {
        if (NTHelper.getString("/sourcePickUp", "none").equals("ground")) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isSourceFeed() {
        if (NTHelper.getString("/sourcePickUp", "none").equals("source")) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        if (RobotState.isTeleop() && !RobotState.isDisabled()) { // if this is wrong its amory's fault (if you are
                                                                 // confused as to who amory is I
            // am too)
            if (visionSubsystem.seesNote()) {
                ledController.set("orange");
            } else if (isGroundPickUp()) {
                ledController.set("green");
            } else if (isSourceFeed()) {
                ledController.set("purple");
            } else {
                ledController.set("dark");
            }
        } else if (RobotState.isAutonomous() && !RobotState.isDisabled()) {
            ledController.set("rainbow");
        } else {
            // System.out.println("disabled");
            if (RobotState.isDisabled() && visionSubsystem.seesAprilTag()) {
                ledController.set("yellow");
            } else {
                ledController.set("dark");
            }
        }
        ledController.run();

    }

}