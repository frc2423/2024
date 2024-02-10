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
        return Commands.run(() -> {
            shooter.shooterOn();
        }).withTimeout(shooter.isDoneSec);
    }

    public Command stopIt() {
        var command = Commands.run(() -> shooter.everythingOffPlease(), shooter);
        command.setName("Stop it");
        return command;
    }

    public Command shoot() {
        return Commands.run(() -> {
            shooter.moveFeederMotor();
        }).withTimeout(shooter.isDoneShoot);
    }

    public Command revAndShoot() {
        return Commands.sequence(
                rev(),
                shoot());
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
