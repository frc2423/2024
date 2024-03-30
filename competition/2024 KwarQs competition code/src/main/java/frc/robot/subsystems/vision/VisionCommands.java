package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionCommands{
    private VisionSubsystem vision;
    private SwerveSubsystem drivebase;


    public VisionCommands(VisionSubsystem vision, SwerveSubsystem drivebase){
        this.vision = vision;
        this.drivebase = drivebase;
    }

    public Command noteAutoAlign() {
        var command = Commands.runOnce(() -> drivebase.actuallyLookAngle(vision.getTurn()));
        command.setName("note auto rotate align");
        return command;
      }
    

}


