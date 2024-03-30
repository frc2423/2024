package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

public class VisionCommands{
    private VisionSubsystem vision;
    private SwerveSubsystem drivebase;
    private IntakeSubsystem intake;

    public VisionCommands(VisionSubsystem vision, SwerveSubsystem drivebase, IntakeSubsystem intake){
        this.vision = vision;
        this.drivebase = drivebase;
        this.intake = intake;
    }

    public Command noteAutoAlign() {
        var command = Commands.run(() -> drivebase.actuallyLookAngle(vision.getTurn()));
        command.setName("note auto rotate align");
        return command;
    }

    public Command noteAutoAlignPickUp() {
        Command drive = Commands.runOnce(()->drivebase.drive(drivebase.getTargetSpeeds(.3, 0.0, Rotation2d.fromDegrees(0))));
        Command command = Commands.sequence( 
        noteAutoAlign(),
        drive
        ).until(() -> intake.isBeamBroken());
        return command;
    }
    

}


