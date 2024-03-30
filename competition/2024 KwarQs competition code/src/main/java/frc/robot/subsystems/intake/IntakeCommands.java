package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterAngleCommands;
import frc.robot.subsystems.shooter.ShooterCommands;

public class IntakeCommands {

    private IntakeSubsystem intake;
    private ShooterAngleCommands shooterAngle;

    public IntakeCommands(IntakeSubsystem intake, ShooterAngleCommands shooterAngle) {
        this.intake = intake;
        this.shooterAngle = shooterAngle;
    }

    public Command intakeDown() {
        var command = Commands.run(() -> intake.extend(), intake);
        command.setName("Intake Down");
        return command;
    }

    public Command intakeUp() {
        var command = Commands.run(() -> {
            intake.retract();
            System.out.println("INTAKE UP");

        }, intake);
        command.setName("Intake Up");
        return command;
    }

    public Command intakeIntake(double speed) {
        var command = Commands.run(() -> intake.intake(speed), intake);
        command.setName("Intake Slurp");
        return command;
    }

    public Command intakeIntake() {
        var command = Commands.run(() -> intake.intake(), intake);
        command.setName("Intake Slurp");
        return command;
    }

    public Command intakeAndUp() {
        Command intakeDown = Commands.runOnce(intake::extend);
        Command intakeStart = intakeIntake().until(intake::isBeamBroken);
        Command intakeStop = Commands.runOnce(intake::beltStop);
        Command intakeRetract = Commands.runOnce(intake::retract);
        Command intakeUp = Commands.runOnce(intake::retract);
        Command wait = Commands.waitSeconds(1);
        Command bestSequence = Commands.sequence(intakeDown, intakeStart.withTimeout(20), intakeStop, intakeUp, wait);
        bestSequence.setName("Intake Slurp");
        return bestSequence;
    }

    public Command intakeOuttake() {
        var command = Commands.run(() -> intake.outtake(), intake);
        command.setName("Intake Spit");
        return command;
    }

    public Command intakeInWithRevCommand() {
        var command = intakeIntake().withTimeout(0.5)
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
        Command intakeStart = intakeIntake().until(intake::isBeamBroken);
        Command intakeStop = Commands.runOnce(intake::beltStop);
        Command intakeRetract = Commands.runOnce(intake::retract);
        Command shooterHandOff = shooterAngle.handOffAngleCommand();

        Command sequence = Commands.sequence(shooterHandOff, intakeStart, intakeStop, intakeRetract);
        sequence.setName("Intake untill");
        return sequence;
    }

    public Command intakeSequence() {
        Command intakeDown = Commands.runOnce(intake::extend);
        Command intakeStart = intakeIntake().until(intake::isBeamBroken);
        Command intakeStop = Commands.runOnce(intake::beltStop);
        Command intakeUp = Commands.runOnce(intake::retract);
        Command wait = Commands.waitSeconds(1);
        ;
        Command bestSequence = Commands.sequence(intakeDown, intakeStart.withTimeout(20), intakeStop, intakeUp, wait);
        bestSequence.setName("Intake Sequence");
        return bestSequence;
    }

    public Command beltStopCommand() {
        var command = Commands.runOnce(intake::beltStop, intake);
        return command;
    }

}