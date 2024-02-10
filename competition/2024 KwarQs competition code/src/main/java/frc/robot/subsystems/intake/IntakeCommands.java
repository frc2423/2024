package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

    private IntakeSubsystem intake;

    public IntakeCommands(IntakeSubsystem intake) {
        this.intake = intake;
    }

    public Command intakeDown() {
        var command = Commands.run(() -> intake.extend(), intake);
        command.setName("Intake Down");
        return command;
    }

    public Command intakeUp() {
        var command = Commands.run(() -> intake.retract(), intake);
        command.setName("Intake Up");
        return command;
    }

    public Command intakeIntake() {
        var command = Commands.run(() -> intake.intake(), intake);
        command.setName("Intake Slurp");
        return command;
    }

    public Command intakeOuttake() {
        var command = Commands.run(() -> intake.outtake(), intake);
        command.setName("Intake Barf");
        return command;
    }

    public Command intakeInWithRevCommand() {
        return Commands.run(() -> {
            intakeIntake();
        }).withTimeout(intake.isDoneSec);
    }

    public Command intakeOutWithShoot() {
        return Commands.run(() -> {
            intakeIntake();
        }).withTimeout(intake.isDoneShoot);
    }
}
