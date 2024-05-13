package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NTHelper;
import frc.robot.PoseTransformUtils;

public class SwerveCommands {

    private SwerveSubsystem swerve;

    Rotation2d specialAngle = new Rotation2d();

    private MedianFilter currentAngleFilter = new MedianFilter(5);

    public SwerveCommands(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public static double normalizedAngle(double currentAngleDegrees) {
        double angle = currentAngleDegrees % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public static double getNormalizedAngleDiff(double a, double b) {
        double diff = Math.abs(a - b);
        if (diff < 180) {
            return diff;
        } else if (a < b) {
            return Math.abs(a + 360 - b);
        } else {
            return Math.abs(b + 360 - a);
        }
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

    public Command lookAtTarget(Pose2d targetAngle, Rotation2d offset) { // to
        var command = Commands.sequence(
                Commands.runOnce(currentAngleFilter::reset),
                Commands.run(() -> {
                    Pose2d transformedPose = PoseTransformUtils.transformXRedPose(targetAngle);
                    specialAngle = swerve.getLookAngle(transformedPose).plus(offset);
                    swerve.actuallyLookAngle(specialAngle);
                }, swerve).until(() -> {
                    double desiredAngle = normalizedAngle(specialAngle.getDegrees());
                    double currentAngle = currentAngleFilter
                            .calculate(normalizedAngle(swerve.getHeading().getDegrees()));
                    double angleDiff = getNormalizedAngleDiff(desiredAngle, currentAngle);
                    return angleDiff < 3;
                }),
                Commands.run(() -> {
                    Pose2d transformedPose = PoseTransformUtils.transformXRedPose(targetAngle);
                    specialAngle = swerve.getLookAngle(transformedPose).plus(offset);
                    swerve.actuallyLookAngle(specialAngle);
                }, swerve).withTimeout(.5),
                Commands.runOnce(() -> {
                    swerve.stop();
                }));

        command.setName("setLookAngle");
        return command;
    }

    public Command lookAtTargetButStillMove(Pose2d targetAngle, Rotation2d offset) { // to
        var command = Commands.sequence(
                Commands.runOnce(currentAngleFilter::reset),
                Commands.run(() -> {
                    Pose2d transformedPose = PoseTransformUtils.transformXRedPose(targetAngle);
                    specialAngle = swerve.getLookAngle(transformedPose).plus(offset);
                    swerve.actuallyLookAngleButMove(specialAngle);
                }, swerve).until(() -> {
                    double desiredAngle = normalizedAngle(specialAngle.getDegrees());
                    double currentAngle = currentAngleFilter
                            .calculate(normalizedAngle(swerve.getHeading().getDegrees()));
                    double angleDiff = getNormalizedAngleDiff(desiredAngle, currentAngle);
                    return angleDiff < 3;
                }),
                Commands.run(() -> {
                    Pose2d transformedPose = PoseTransformUtils.transformXRedPose(targetAngle);
                    specialAngle = swerve.getLookAngle(transformedPose).plus(offset);
                    swerve.actuallyLookAngleButMove(specialAngle);
                }, swerve));

        command.setName("setLookAngleButMove");
        return command;
    }

    public Command autoAlignShootCommand(Pose2d pose) {

        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation
        Pose2d targetPose = PoseTransformUtils.transformXRedPose(pose);

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

    public Command autoAlignAmpCommand(Pose2d pose) {


        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation
        Pose2d targetPose = PoseTransformUtils.transformXRedPose(pose);

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

        pathfindingCommand.setName("Align to Amp");

        return pathfindingCommand;
    }

    public Command autoAlignSourceMiddleCommand(Pose2d pose) {


        // Since we are using a holonomic drivetrain, the rotation component of this
        // pose
        // represents the goal holonomic rotation
        Pose2d targetPose = PoseTransformUtils.transformXRedPose(pose);

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

        pathfindingCommand.setName("Align to Source Middle");

        return pathfindingCommand;
    }

  
}
