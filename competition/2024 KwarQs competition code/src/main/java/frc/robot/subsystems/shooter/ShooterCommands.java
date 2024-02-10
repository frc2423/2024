package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ShooterCommands {

    private ShooterSubsystem shooter;
    private IntakeCommands intake;

    public ShooterCommands(ShooterSubsystem shooter, IntakeCommands intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    public Command rev() {
        var command = Commands.run(() -> {
            shooter.shooterOn();
        }).withTimeout(shooter.isDoneSec);
        command.setName("Revvvvv");
        return command;
    }

    public Command stopIt() {
        var command = Commands.run(() -> shooter.everythingOffPlease(), shooter);
        command.setName("Stop it");
        return command;
    }

    public Command shoot() {
        var command = Commands.run(() -> {
            shooter.moveFeederMotor();
        }, shooter).withTimeout(shooter.isDoneShoot);
        command.setName("Shoot");
        return command;
    }

    public Command revAndShoot() {
        var command = Commands.sequence(
                rev(),
                shoot());
        command.setName("Revvvvv and Shoot");
        return command;
    }

    public Command shooterCommand() {
        Command shooterCommand = Commands.sequence(
        Commands.parallel(rev(),intake.intakeInWithRevCommand()),
        Commands.parallel(shoot(),intake.intakeOutWithShoot())
        );
        shooterCommand.setName("Rev,Intake,Shoot");
        return shooterCommand;
    }


}
