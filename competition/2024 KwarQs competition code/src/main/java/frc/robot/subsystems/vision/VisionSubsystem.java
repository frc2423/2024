package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Vision;

public class VisionSubsystem extends SubsystemBase {
    private Vision visionInterface = new Vision();
    public double getLatestId = 0;

    public VisionSubsystem() {
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> pose = visionInterface.getEstimatedGlobalPose();
    }

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        Optional<EstimatedRobotPose> pose = visionInterface.getEstimatedGlobalPose();
        return pose;
    }

    public Optional<Matrix<N3, N1>> getStandardDeviations() {
        Optional<EstimatedRobotPose> pose = getEstimatedRobotPose();
        if (pose.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(visionInterface.getEstimationStdDevs(pose.get().estimatedPose.toPose2d()));
    }

    public double getTimestampSeconds() {
        return visionInterface.getLatestResult().getTimestampSeconds();

    }

    public void simulationPeriodic(Pose2d pose) {
        visionInterface.simulationPeriodic(pose);
    }

    public Optional<Transform3d> getLatestResult() {
        if (!visionInterface.getLatestResult().hasTargets()) {
            return null;
        }
        PhotonTrackedTarget getLatestResult = visionInterface.getLatestResult().getBestTarget();
        if (getLatestResult != null) {
            getLatestId = getLatestResult.getFiducialId();
            return Optional.ofNullable(getLatestResult.getBestCameraToTarget());
        }
        return null;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // This is used to add things to NetworkTables
        super.initSendable(builder);

        builder.addBooleanProperty("has targets", () -> visionInterface.getLatestResult().hasTargets(), null);
        builder.addBooleanProperty("is connected", () -> visionInterface.getCamera().isConnected(), null);
        builder.addIntegerProperty("pipeline index", () -> visionInterface.getCamera().getPipelineIndex(), null);
    }
}
