package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KwarqsLed extends SubsystemBase {
    private LedController ledController = new LedController(36); // 36 on each side

    public KwarqsLed() {
        ledController.add("yellow", new Yellow());
        ledController.add("purple", new Purple());
        ledController.add("green", new Green());
        ledController.add("dark", new Dark());

        setDefaultCommand(disable());
    }

    public Command disable() {
        var command = Commands.run(()-> {
            ledController.set("dark");
        });
        command.addRequirements(this);
        return command;
    }

    public Command setYellow() {
        var command = Commands.run(()-> {
            ledController.set("yellow");
        });
        command.addRequirements(this);
        return command;
    } 

    public Command setPurple() {
       var command = Commands.run(()-> {
            ledController.set("purple");
        });
        command.addRequirements(this);
        return command;
    } 

    public Command setGreen() {
        var command = Commands.run(()-> {
            System.out.println("!!!!!!!!");
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