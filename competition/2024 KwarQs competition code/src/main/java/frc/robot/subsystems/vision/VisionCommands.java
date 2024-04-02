package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionCommands {
    private VisionSubsystem vision;
    private SwerveSubsystem drivebase;
    private IntakeSubsystem intake;
    private IntakeCommands intakeCommands;

    public VisionCommands(VisionSubsystem vision, SwerveSubsystem drivebase, IntakeSubsystem intake,
            IntakeCommands intakeCommands) {
        this.vision = vision;
        this.drivebase = drivebase;
        this.intake = intake;
        this.intakeCommands = intakeCommands;
    }

    public Command notePoseAutoAlign() {
        Command command = Commands.run(() -> drivebase.actuallyLookAngle(vision.getTurn()))
                .until(() -> vision.isAlignedNote());
        command.setName("note auto rotate align pose");
        return command;
    }

    public Command noteAutoAlign() {
        Command turn = Commands.run(() -> drivebase.turn(-vision.getNoteYaw() * .1), drivebase, vision);
        Command command = turn.until(() -> vision.isAlignedNote());
        command.setName("note auto rotate align");
        return command;
    }

    public Command noteAutoAlignPickUp() {
        Command turn = Commands.run(() -> drivebase.turn(-vision.getNoteYaw() * .1), drivebase, vision);
        Command driveAndTurn = Commands.run(() -> drivebase.turnAndGo(.8, -vision.getNoteYaw() * .1), drivebase, vision);
        Command command = Commands.sequence(
                turn.until(() -> vision.isAlignedNote()),
                // intakeCommands.intakeDown(),
                Commands.parallel(driveAndTurn, intakeCommands.intakeIntake())).until(() -> intake.isBeamBroken());
        return command;
    }

}
