package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SwerveCommands {

    private SwerveSubsystem swerve;

    public SwerveCommands(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public Command setSlowMaxSpeed() {
        var command = Commands.run(() -> {
            swerve.setSlowMaxSpeed();
        });
        command.setName("SLOW");
        return command;
    }

    public Command setHighMaxSpeed() {
        var command = Commands.run(() -> {
            swerve.setHighMaxSpeed();
        });
        command.setName("BIG GOOOOOOOO");
        return command;
    }

<<<<<<< HEAD
    public Command autoAlignShootCommand() {
        var command = swerve.autoAlignShoot();
        command.setName("Align to Shoot");
        return command;
    }


=======
>>>>>>> 27dc79d4159e73eef62e872555defe20f9cd7156
}
