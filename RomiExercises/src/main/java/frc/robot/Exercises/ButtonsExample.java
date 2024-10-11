package frc.robot.Exercises;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

public class ButtonsExample {
    public XboxController controller;
    public Drivetrain drive;
    public ButtonsExample(Drivetrain drivetrain, XboxController controller) {
        drive = drivetrain;
        this.controller = controller;
    }

    public void addBindings() {
        new JoystickButton(controller, XboxController.Button.kY.value).onTrue(
            new DriveDistance(-0.5, 1, drive)
        );
    }

}