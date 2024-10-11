package frc.robot.Exercises;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveTime;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.TurnTime;
import frc.robot.subsystems.Drivetrain;
/*
 * TODO: Move the romi forward when Y is pressed and backwards when X is pressed
 * 
 * Change the 
 * 
 */

public class ForwardBackButtons {
    public XboxController controller;
    public Drivetrain drive;
    public ForwardBackButtons(Drivetrain drivetrain, XboxController controller) {
        drive = drivetrain;
        this.controller = controller;
    }

    public void addBindings() {

    }

}