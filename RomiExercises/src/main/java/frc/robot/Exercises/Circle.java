package frc.robot.Exercises;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.TurnTime;
import frc.robot.subsystems.Drivetrain;
/*
 * TODO: Move the romi in a Circle during autonomous, make sure to select Circle in the autonomous selection
 * 
 */

public class Circle extends SequentialCommandGroup {
    public Circle(Drivetrain drivetrain) {
        addCommands(

        );
    }
}