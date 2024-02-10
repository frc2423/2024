package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class ShooterCommands {

    private ShooterSubsystem shooter;

    public ShooterCommands(ShooterSubsystem shooter) {
        this.shooter = shooter;
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

}
