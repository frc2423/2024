package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterAngleCommands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionCommands {
    private VisionSubsystem vision;
    private SwerveSubsystem drivebase;
    private IntakeSubsystem intake;
    private IntakeCommands intakeCommands;
    private ShooterAngleCommands shooterAngleCommands;

    public VisionCommands(VisionSubsystem vision, SwerveSubsystem drivebase, IntakeSubsystem intake,
            IntakeCommands intakeCommands, ShooterAngleCommands shooterAngleCommands) {
        this.vision = vision;
        this.drivebase = drivebase;
        this.intake = intake;
        this.intakeCommands = intakeCommands;
        this.shooterAngleCommands = shooterAngleCommands;
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
        Command turn = Commands.run(() -> drivebase.turn(-vision.getNoteYaw() * .075), drivebase, vision);
        Command driveAndTurn = Commands.run(() -> drivebase.turnAndGo(2, vision.yawAfterAligned()), drivebase, vision);
        Command stopMoving =  Commands.runOnce(() -> drivebase.turnAndGo(0, 0), drivebase);
        Command command = Commands.sequence(
                Commands.parallel(intakeCommands.intakeDown(), shooterAngleCommands.handOffAngleCommand()).withTimeout(.2),
                turn.until(() -> vision.isAlignedNote()),
                // intakeCommands.intakeDown(),
                Commands.parallel(driveAndTurn, intakeCommands.intakeIntake()).until(() -> intake.isBeamBroken()),
                stopMoving);
        return command;
    }
}
