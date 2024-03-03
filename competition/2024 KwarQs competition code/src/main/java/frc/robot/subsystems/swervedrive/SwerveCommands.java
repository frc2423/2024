package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

    public Command autoAlignShootCommand() {

        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(2.34, 5.59, Rotation2d.fromDegrees(0));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                2.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0);// Rotation delay distance in meters. This is how far the robot should travel
                     // before attempting to rotate.

        pathfindingCommand.setName("Align to Shoot");

        return pathfindingCommand;
    }
}
