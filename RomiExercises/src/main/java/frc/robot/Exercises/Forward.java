package frc.robot.Exercises;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.TurnTime;
import frc.robot.subsystems.Drivetrain;
/*
 * TODO: Move the robot forward during autonomous, make sure to select forward in the autonomous selection
 * 
 */

public class Forward extends SequentialCommandGroup {
    public Forward(Drivetrain drivetrain) {
        addCommands(
            
        );
    }
}