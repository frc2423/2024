package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KwarqsLed extends SubsystemBase {
    private LedController ledController = new LedController(64);

    public KwarqsLed() {
        ledController.add("yellow", new Yellow());
        ledController.add("purple", new Purple());
        ledController.add("green", new Green());
        ledController.add("dark", new Dark());

        setDefaultCommand(setYellow());
    }

    public Command disable() {
        var command = Commands.runOnce(()-> {
            ledController.set("dark");
        });
        command.addRequirements(this);
        return command;
    }

    public Command setYellow() {
        var command = Commands.runOnce(()-> {
            ledController.set("yellow");
        });
        command.addRequirements(this);
        return command;
    } 

    public Command setPurple() {
       var command = Commands.runOnce(()-> {
            ledController.set("purple");
        });
        command.addRequirements(this);
        return command;
    } 

    public Command setGreen() {
        var command = Commands.runOnce(()-> {
            ledController.set("green");
        });
        command.addRequirements(this);
        return command;
    } 

    @Override
    public void periodic() {
        ledController.run();
    }
}