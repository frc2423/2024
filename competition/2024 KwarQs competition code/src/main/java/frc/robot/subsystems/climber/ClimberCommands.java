package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterAngleCommands;

public class ClimberCommands {
    
    private ClimberSubsystem climberSubsystem;

     public ClimberCommands(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
    }

    public Command climbStartCommand() {
        var command = Commands.runOnce(() -> climberSubsystem.climbStart());
        command.addRequirements(climberSubsystem);
        command.setName("Start Climb");
        return command;
    }
    
    public Command climbDownCommand() {
        var command = Commands.runOnce(() -> climberSubsystem.climbDown());
        command.addRequirements(climberSubsystem);
        command.setName("Start Climb");
        return command;
    }

    public Command climbStopCommand() {
        var command = Commands.runOnce(() -> climberSubsystem.climbStop());
        command.addRequirements(climberSubsystem);
        command.setName("Stop Climb");
        return command;
    }













}
