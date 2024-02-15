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
        command.setName("Intake Spit");
        return command;
    }

    public Command intakeInWithRevCommand() {
        var command = intakeIntake().withTimeout(0.1)
                .withTimeout(intake.isDoneSec);
        command.setName("Intake In With Rev");
        return command;
    }

    public Command intakeOutWithFeedCommand() {
        var command = intakeOuttake().withTimeout(0.1)
                .withTimeout(intake.isDoneSec);
        command.setName("Intake out Fed");
        return command;
    }

    public Command intakeOutWithShoot() {
        var command = intakeOuttake().withTimeout(intake.isDoneShoot);
        command.setName("Intake Out To Shoot");
        return command;
    }

    public Command intakeIntakeUntil() {
        var command =   intakeIntake().until(intake::isBeamBroken);
        command.setName("Intake untill");
        System.out.println("Hello world!");
        return command;
    }
}
